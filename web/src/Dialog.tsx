import {
  createContext,
  type FormEvent,
  type ReactNode,
  useCallback,
  useContext,
  useEffect,
  useId,
  useState,
} from "react";

export interface DialogOptions {
  title: string;
  message: string;
  confirmLabel?: string;
  cancelLabel?: string;
  tone?: "default" | "danger";
  input?: {
    label: string;
    placeholder?: string;
    required?: boolean;
  };
}

type DialogRequest = (options: DialogOptions) => Promise<string | null>;

interface PendingDialog {
  options: DialogOptions;
  resolve: (result: string | null) => void;
}

const DialogContext = createContext<DialogRequest | null>(null);

export function DialogProvider({ children }: { children: ReactNode }) {
  const [pending, setPending] = useState<PendingDialog | null>(null);
  const requestDialog = useCallback<DialogRequest>((options) =>
    new Promise((resolve) => setPending({ options, resolve })), []);
  const close = useCallback((result: string | null) => {
    if (!pending) return;
    pending.resolve(result);
    setPending(null);
  }, [pending]);

  useEffect(() => {
    if (!pending) return;
    const priorOverflow = document.body.style.overflow;
    document.body.style.overflow = "hidden";
    return () => { document.body.style.overflow = priorOverflow; };
  }, [pending]);

  return <DialogContext.Provider value={requestDialog}>
    {children}
    {pending && <ActionDialog
      options={pending.options}
      onCancel={() => close(null)}
      onConfirm={(value) => close(value)}
    />}
  </DialogContext.Provider>;
}

export function useDialog(): DialogRequest {
  const requestDialog = useContext(DialogContext);
  if (!requestDialog) throw new Error("useDialog must be used inside DialogProvider");
  return requestDialog;
}

function ActionDialog({ options, onCancel, onConfirm }: {
  options: DialogOptions;
  onCancel: () => void;
  onConfirm: (value: string) => void;
}) {
  const [value, setValue] = useState("");
  const titleId = useId();
  const messageId = useId();
  const inputId = useId();
  const inputRequired = options.input?.required === true;
  const confirmDisabled = inputRequired && value.trim().length === 0;

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape") onCancel();
    };
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [onCancel]);

  const submit = (event: FormEvent) => {
    event.preventDefault();
    if (!confirmDisabled) onConfirm(options.input ? value.trim() : "confirmed");
  };

  return <div className="dialog-backdrop" onMouseDown={(event) => {
    if (event.target === event.currentTarget) onCancel();
  }}>
    <form
      aria-describedby={messageId}
      aria-labelledby={titleId}
      aria-modal="true"
      className="action-dialog"
      onSubmit={submit}
      role="dialog"
    >
      <div className="dialog-heading">
        <p className="eyebrow">Operator confirmation</p>
        <h2 id={titleId}>{options.title}</h2>
      </div>
      <p id={messageId}>{options.message}</p>
      {options.input && <label className="dialog-input" htmlFor={inputId}>
        <span>{options.input.label}{inputRequired ? " (required)" : " (optional)"}</span>
        <textarea
          autoFocus
          id={inputId}
          onChange={(event) => setValue(event.target.value)}
          placeholder={options.input.placeholder}
          rows={3}
          value={value}
        />
      </label>}
      <div className="dialog-actions">
        <button autoFocus={!options.input} className="secondary" onClick={onCancel} type="button">
          {options.cancelLabel ?? "Cancel"}
        </button>
        <button
          className={options.tone === "danger" ? "danger" : ""}
          disabled={confirmDisabled}
          type="submit"
        >
          {options.confirmLabel ?? "Continue"}
        </button>
      </div>
    </form>
  </div>;
}
