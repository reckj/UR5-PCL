# AI context folder for students (2026-10-28, late) 🟢

**Status: 🟢 in the student repo** (commit `0b236c2`). Content is derived from
the code as of `7a755a5`; it has to be kept in step with the public API.

## Why

Students use whatever AI assistant they have privately (different models,
chat tools and editor integrations). Out of context those tools know UR
robots and `ur_rtde` in general and will invent framework methods, use
degrees, or write raw URScript. The repo now carries a tool-agnostic knowledge
base the student hands to their assistant.

## What is in the student repo

```
context/
  README.md                 what each file is for, how to load it into chat tools
                            (ChatGPT/Claude Projects, Gems…), editor tools, plain chat
  INSTRUCTIONS.md           system-prompt-style rules for an AI helping here (~7 KB)
  KNOWLEDGE.md              facts: environment, Robot/Workspace API with signatures,
                            safety profiles + numbers, lab robot, viewer, error table (~16 KB)
  framework_source.md       GENERATED: src/ur5e/*.py (no viewer) + function templates +
                            grid_process + docker-compose.yml, one file (~63 KB)
  make_source_snapshot.sh   regenerates framework_source.md; header carries git hash
  ur_rtde/*.txt             ur_rtde 1.6.3 docs (api_reference, guides, examples, faq,
                            installation, introduction, rtde_protocol_guide) as plain text,
                            extracted from the Sphinx HTML main div (bs4 + pandoc -t plain)
  manual/UR5e_User_Manual_en_Global.{pdf,txt}   official UR5e manual, 17 MB PDF + 720 KB
                            pdftotext -layout text (section numbers preserved)
AGENTS.md                   root pointer → context/ (Codex, Cursor, Gemini CLI, Aider…)
CLAUDE.md                   root pointer (Claude Code)
.github/copilot-instructions.md   root pointer (Copilot in VS Code)
```

`README.md` links the folder; `docs/USAGE.md` §8 explains the workflow.

## Decisions

- **Tool-agnostic plain Markdown/text**, no vendor rule formats except the
  three root discovery files that cost nothing. No `.cursor/rules`,
  `.windsurfrules` etc. — AGENTS.md is read by most of them now.
- **Two layers**: INSTRUCTIONS (how to behave — framework only, fractions of
  the taught box, m/rad, sim first, never bypass `SafetyViolation`, no invented
  methods, no safety boilerplate beyond the docs) and KNOWLEDGE (facts). Kept
  separate because chat tools have a custom-instructions slot and a file slot.
- **`framework_source.md` is committed** although generated, so a student can
  upload it to a chat tool without running anything. Regenerate after every
  sync (`sync_to_student_repo.sh` prints the command; RELEASING §D/§F updated).
- **Manual included as PDF + text.** The text is what the AIs can actually
  search (~200k tokens); the PDF keeps the figures. Public-repo redistribution
  of the UR manual is John's call — alternatives are text-only or a link.
- The context folder is **owned by the student repo** (not synced). API
  changes mean editing `context/KNOWLEDGE.md` §4 by hand.

## Open

- Verify with one or two real tools (ChatGPT Project, Cursor) that the
  instructions actually suppress invented methods; tighten INSTRUCTIONS from
  what students report in week 1.
- Possibly a `context/EXAMPLES.md` with 3–4 worked "process → script"
  decompositions once student projects exist.
