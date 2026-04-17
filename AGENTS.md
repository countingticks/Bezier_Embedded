- Prefer minimal diffs.
- Change only the requested function or selected range.
- Preserve formatting, comments, and import order unless correctness requires otherwise.
- Do not refactor unrelated code.
- Do not rewrite whole blocks when a local edit will do.

## graphify

This project has a graphify knowledge graph at graphify-out/.

Rules:
- Before answering architecture or codebase questions, read graphify-out/GRAPH_REPORT.md for god nodes and community structure
- If graphify-out/wiki/index.md exists, navigate it instead of reading raw files
- After modifying code files in this session, run `graphify update .` to keep the graph current (AST-only, no API cost)
