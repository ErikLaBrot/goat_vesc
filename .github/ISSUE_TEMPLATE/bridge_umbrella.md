---
name: Release meta
about: Define a release outcome while the milestone tracks the work items.
title: ""
labels: []
assignees: []
---

## Goal

Describe the release outcome this meta issue is defining.

For release meta issues, the title may be the release qualifier itself, for
example `goat_vesc_driver v1.0`.

## Release Definition

- This issue is a lightweight release brief, not the authoritative work tracker.
- The milestone is the authoritative release bucket for included work items.
- Local planning or backlog docs may mirror this release definition, but GitHub
  remains authoritative if they differ.
- Resolving this issue means the release definition, requirement references, and
  accepted exceptions are current for the milestone.

## Milestone

- Milestone: `goat_vesc_driver v1.0`

## Acceptance Criteria

- [ ] List the release-meaningful outcome.
- [ ] List the evidence needed for approval.
- [ ] List the required doc or checklist updates.
- [ ] Confirm the linked milestone contains the actual release work items.

## Checklist And Docs

- [ ] Link the milestone.
- [ ] Link `BRIDGE_READINESS_CHECKLIST.md`.
- [ ] Link any release summary docs if needed.

## Known Accepted Exceptions

- None currently.

## Requirement IDs

- `BRIDGE-...`

## Notes For Local Execution

- Plan locally against this issue before implementation.
- Keep each release work item to one focused branch and one PR.
