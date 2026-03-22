# Repository Guidelines

## Project Structure & Module Organization
This repository is currently document-centric and contains a single workbook:
- `PN0031 250411 zh-CN to en 2026-01-31 02-15-55.xlsx`: primary translation/data artifact.

Keep new files organized by purpose as the project grows:
- `docs/` for reference notes and change logs.
- `assets/` for supporting exports (CSV, screenshots).
- `archive/` for timestamped legacy workbook versions.

## Build, Test, and Development Commands
There is no code build pipeline yet. Use lightweight validation steps:
- `ls -la` to confirm expected files before and after edits.
- `file *.xlsx` to verify workbook format.
- `cp "PN0031 250411 zh-CN to en 2026-01-31 02-15-55.xlsx" archive/<new-name>.xlsx` to create a backup before major edits.

If automation is introduced later, document exact commands here (for example, linting exported CSV or schema checks).

## Coding Style & Naming Conventions
For workbook and asset names, use clear, sortable patterns:
- `<project> <source> to <target> YYYY-MM-DD HH-MM-SS.xlsx`
- Example: `PN0031 zh-CN to en 2026-02-16 10-30-00.xlsx`

Guidelines:
- Prefer ISO-style dates (`YYYY-MM-DD`).
- Use lowercase language tags (`zh-CN`, `en`).
- Avoid ambiguous names like `final.xlsx` or `new.xlsx`.

## Testing Guidelines
No automated test framework is configured. Validate changes manually:
- Confirm row/column counts did not shift unexpectedly.
- Spot-check translated cells for formatting, formulas, and encoding.
- Re-open the workbook in a second editor to catch compatibility issues.

## Commit & Pull Request Guidelines
Git history is not available in this directory, so adopt a standard convention:
- Commit format: `type(scope): summary` (for example, `docs(workbook): normalize filename timestamps`).
- Keep commits focused to one logical change.
- PRs should include: purpose, changed files, validation steps performed, and before/after screenshots for visible sheet changes.

## Security & Configuration Tips
- Do not commit sensitive or customer-identifying data without approval.
- Keep raw source files unchanged; edit working copies and preserve originals in `archive/`.
