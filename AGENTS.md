# Agent Guide

### Rules

- Use ASD-STE100 Simplified Technical English. Keep answers short and to the
  point.
- Be honest about commands run, observed state, failures, and uncertainty.
- Write detailed commits with lines below 80 characters. Explain behavior,
  validation, and follow-up context; verify the final message after committing.
- Do not use repeated `git commit -m` arguments for adjacent body lines. Each
  argument creates a new paragraph and can make the message double-spaced. Use
  one prepared message, then inspect `git show -s --format=%B` for line width
  and unintended blank lines.
