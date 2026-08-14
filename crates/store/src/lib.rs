use anyhow::{Context, Result};
use parking_lot::Mutex;
use rusqlite::{params, Connection, OptionalExtension};
use serde::{de::DeserializeOwned, Serialize};
use std::path::Path;
use std::sync::Arc;
use uuid::Uuid;
use velocitypnp_domain::{
    BoardDesign, CalibrationSnapshot, Feeder, InspectionReport, Job, MachineEvent, Operation,
};

#[derive(Clone)]
pub struct Store {
    connection: Arc<Mutex<Connection>>,
}

impl Store {
    pub fn open(path: &Path) -> Result<Self> {
        let connection = Connection::open(path)
            .with_context(|| format!("opening database {}", path.display()))?;
        let store = Self {
            connection: Arc::new(Mutex::new(connection)),
        };
        store.migrate()?;
        Ok(store)
    }

    pub fn memory() -> Result<Self> {
        let store = Self {
            connection: Arc::new(Mutex::new(Connection::open_in_memory()?)),
        };
        store.migrate()?;
        Ok(store)
    }

    fn migrate(&self) -> Result<()> {
        self.connection.lock().execute_batch(
            r#"
            PRAGMA journal_mode = WAL;
            PRAGMA foreign_keys = ON;
            CREATE TABLE IF NOT EXISTS schema_version (
                version INTEGER PRIMARY KEY,
                applied_at TEXT NOT NULL
            );
            INSERT OR IGNORE INTO schema_version(version, applied_at)
                VALUES (1, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'));
            CREATE TABLE IF NOT EXISTS entities (
                kind TEXT NOT NULL,
                id TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                document TEXT NOT NULL,
                PRIMARY KEY(kind, id)
            );
            CREATE TABLE IF NOT EXISTS event_journal (
                row_id INTEGER PRIMARY KEY AUTOINCREMENT,
                machine_sequence INTEGER NOT NULL,
                occurred_at TEXT NOT NULL,
                kind TEXT NOT NULL,
                payload TEXT NOT NULL
            );
            CREATE INDEX IF NOT EXISTS event_journal_occurred_at
                ON event_journal(occurred_at);
            "#,
        )?;
        Ok(())
    }

    pub fn put_board(&self, board: &BoardDesign) -> Result<()> {
        self.put("board", board.id, board)
    }

    pub fn list_boards(&self) -> Result<Vec<BoardDesign>> {
        self.list("board")
    }

    pub fn get_board(&self, id: Uuid) -> Result<Option<BoardDesign>> {
        self.get("board", id)
    }

    pub fn put_operation(&self, operation: &Operation) -> Result<()> {
        self.put("operation", operation.id, operation)
    }

    pub fn get_operation(&self, id: Uuid) -> Result<Option<Operation>> {
        self.get("operation", id)
    }

    pub fn put_calibration(&self, calibration: &CalibrationSnapshot) -> Result<()> {
        self.put("calibration", calibration.id, calibration)
    }

    pub fn list_calibrations(&self) -> Result<Vec<CalibrationSnapshot>> {
        self.list("calibration")
    }

    pub fn get_calibration(&self, id: Uuid) -> Result<Option<CalibrationSnapshot>> {
        self.get("calibration", id)
    }

    pub fn put_feeder(&self, feeder: &Feeder) -> Result<()> {
        self.put("feeder", feeder_id(feeder), feeder)
    }

    pub fn list_feeders(&self) -> Result<Vec<Feeder>> {
        self.list("feeder")
    }

    pub fn put_job(&self, job: &Job) -> Result<()> {
        self.put("job", job.id, job)
    }

    pub fn list_jobs(&self) -> Result<Vec<Job>> {
        self.list("job")
    }

    pub fn get_job(&self, id: Uuid) -> Result<Option<Job>> {
        self.get("job", id)
    }

    pub fn put_inspection(&self, inspection: &InspectionReport) -> Result<()> {
        self.put("inspection", inspection.id, inspection)
    }

    pub fn list_inspections(&self) -> Result<Vec<InspectionReport>> {
        self.list("inspection")
    }

    pub fn get_inspection(&self, id: Uuid) -> Result<Option<InspectionReport>> {
        self.get("inspection", id)
    }

    pub fn append_event(&self, event: &MachineEvent) -> Result<()> {
        self.connection.lock().execute(
            "INSERT INTO event_journal(machine_sequence, occurred_at, kind, payload) VALUES (?1, ?2, ?3, ?4)",
            params![event.sequence, event.at.to_rfc3339(), event.kind, serde_json::to_string(&event.payload)?],
        )?;
        Ok(())
    }

    pub fn recent_events(&self, limit: u32) -> Result<Vec<MachineEvent>> {
        let connection = self.connection.lock();
        let mut statement = connection.prepare(
            "SELECT machine_sequence, occurred_at, kind, payload FROM event_journal ORDER BY row_id DESC LIMIT ?1",
        )?;
        let rows = statement.query_map([limit.clamp(1, 10_000)], |row| {
            let at: String = row.get(1)?;
            let payload: String = row.get(3)?;
            Ok(MachineEvent {
                sequence: row.get(0)?,
                at: chrono::DateTime::parse_from_rfc3339(&at)
                    .map(|value| value.with_timezone(&chrono::Utc))
                    .map_err(|error| {
                        rusqlite::Error::FromSqlConversionFailure(
                            1,
                            rusqlite::types::Type::Text,
                            Box::new(error),
                        )
                    })?,
                kind: row.get(2)?,
                payload: serde_json::from_str(&payload).map_err(|error| {
                    rusqlite::Error::FromSqlConversionFailure(
                        3,
                        rusqlite::types::Type::Text,
                        Box::new(error),
                    )
                })?,
            })
        })?;
        let mut events = rows.collect::<rusqlite::Result<Vec<_>>>()?;
        events.reverse();
        Ok(events)
    }

    fn put<T: Serialize>(&self, kind: &str, id: Uuid, value: &T) -> Result<()> {
        self.connection.lock().execute(
            r#"INSERT INTO entities(kind, id, updated_at, document)
               VALUES (?1, ?2, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'), ?3)
               ON CONFLICT(kind, id) DO UPDATE SET
                 updated_at = excluded.updated_at,
                 document = excluded.document"#,
            params![kind, id.to_string(), serde_json::to_string(value)?],
        )?;
        Ok(())
    }

    fn get<T: DeserializeOwned>(&self, kind: &str, id: Uuid) -> Result<Option<T>> {
        let document: Option<String> = self
            .connection
            .lock()
            .query_row(
                "SELECT document FROM entities WHERE kind = ?1 AND id = ?2",
                params![kind, id.to_string()],
                |row| row.get(0),
            )
            .optional()?;
        document
            .map(|value| serde_json::from_str(&value).context("decoding stored entity"))
            .transpose()
    }

    fn list<T: DeserializeOwned>(&self, kind: &str) -> Result<Vec<T>> {
        let connection = self.connection.lock();
        let mut statement = connection
            .prepare("SELECT document FROM entities WHERE kind = ?1 ORDER BY updated_at DESC")?;
        let rows = statement.query_map([kind], |row| row.get::<_, String>(0))?;
        rows.map(|row| {
            let document = row?;
            serde_json::from_str(&document).map_err(|error| {
                rusqlite::Error::FromSqlConversionFailure(
                    0,
                    rusqlite::types::Type::Text,
                    Box::new(error),
                )
            })
        })
        .collect::<rusqlite::Result<Vec<_>>>()
        .map_err(Into::into)
    }
}

fn feeder_id(feeder: &Feeder) -> Uuid {
    match feeder {
        Feeder::Strip { id, .. } | Feeder::Drag { id, .. } | Feeder::Photon { id, .. } => *id,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use velocitypnp_domain::BoardDesign;

    #[test]
    fn board_round_trip() {
        let store = Store::memory().unwrap();
        let board = BoardDesign {
            id: Uuid::new_v4(),
            name: "fixture".into(),
            source_format: "kicad_pcb".into(),
            source_version: Some(20240108),
            width_mm: Some(10.0),
            height_mm: Some(20.0),
            placements: vec![],
            warnings: vec![],
        };
        store.put_board(&board).unwrap();
        assert_eq!(store.get_board(board.id).unwrap().unwrap().name, "fixture");
    }
}
