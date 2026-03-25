## Project & license

Our robot, codenamed roomba, has its full code publicly available. It is licensed under the MIT License — feel free to use or modify the code in any way you want. Keep the license file in the repository and preserve the copyright notice when redistributing.

## Contributing

You cannot commit directly to the `main` branch. Instead, create a new branch for your work and then open a pull request to merge into `main`.

To merge your pull request the following requirements must be met:

- Your branch must compile. GitHub Actions runs the build and tests for pull requests; CI must pass before merging.
- Your pull request must be approved by at least one of the following maintainers: `Caedmon`, `Tullysaurus`, or `theron17`.

Note: If your change fixes a bug, reference the issue in your PR description (for example, `Fixes #42`) so it will auto-close when merged.

---

## Code style (Java)

These guidelines are tuned for a Java-based WPILib/FRC project. They are intentionally simple so automated formatters can be applied.

- Indentation: 4 spaces. Never use tabs.
- Line length: wrap at 100 characters where reasonable.
- File encoding: UTF-8. Ensure a single trailing newline at end-of-file.
- Brace style: K&R (opening brace stays on the same line as the declaration).

  Example:

  public class Example {
      public void doSomething() {
          if (condition) {
              // ...
          } else {
              // ...
          }
      }
  }

- Naming:
  - Classes & enums: PascalCase (e.g., `DriveSubsystem`, `AutoMode`).
  - Methods & variables: camelCase (e.g., `calculateSpeed`, `leftMotor`).
  - Constants: ALL_CAPS_WITH_UNDERSCORES (public static final). Use `TUNER_CONSTANT` style for generated constants.
  - Packages: all-lowercase, reverse domain style (already used in `frc.robot`).

- Access modifiers: Prefer explicit accessibility (e.g., `private`, `protected`, `public`) rather than package-private unless intentional.
- Immutability: Use `final` for variables/fields that should not be reassigned.

- Imports:
  - Do not use wildcard imports (no `import java.util.*`).
  - Order imports logically (e.g., java.*, javax.*, third-party libs, then project packages), with a blank line before project/imports if helpful.
  - Group static imports together and keep them separate from normal imports.

- Comments & docs:
  - Public classes, public methods, and complex logic should have Javadoc comments using `/** ... */`.
  - Inline comments should be sparing and explain "why" more than "what".

- Formatting & whitespace:
  - Spaces around binary operators and after commas.
  - No trailing whitespace.
  - Prefer expressive variable/method names over long comments when possible.

- Recommended idioms for robot code:
  - Keep state in subsystems; prefer composition over large monolithic classes.
  - Keep commands small and focused (single responsibility).
  - Use WPILib conventions for command-based structure (subsystems, commands, RobotContainer wiring).

## Automated formatting and checks

We recommend using an automatic Java formatter and static checks to keep style consistent. Suggested tools:

- google-java-format (formatter)
- Checkstyle or Spotless (to enforce import order and formatting in CI)
- SpotBugs or Error Prone (optional static analysis)

Example Gradle tasks that are usually available in this project:

```bash
./gradlew build        # runs compilation and tests
./gradlew check        # runs static checks if configured
```

If you add a formatter or linter, update the build so CI can fail early on style violations.

## Commit messages

- Keep commit messages concise and focused; use the imperative tense (e.g., "Add intake subsystem tests").
- Include a short body when needed to explain reasoning or trade-offs.

## Branching and pull requests

- Branch naming: use descriptive names. Examples:
  - `feature/<short-description>`
  - `fix/<short-description>`
  - `chore/<short-description>`

- PR checklist (items to include in your Pull Request description):
  - Short summary of the change.
  - The rationale for the change (why).
  - What was changed (high-level list of files / components).
  - How it was tested locally (unit tests, simulation, robot run). Include console output / logs if relevant.
  - Link to any related issue(s) (use `#<issue-number>` if in-repo).

- PR template (suggested structure):

  Summary:

  Changes:

  Testing:

  Related issues:

- Review process:
  - Open a pull request against the `main` branch (or the branch indicated by maintainers).
  - Request review from at least one team member. Larger changes should get two reviewers.
  - Address review comments with follow-up commits. Small fixes can be squashed by the merger at their discretion.
  - Ensure CI passes before merge.

- Merging:
  - Pull requests require explicit approval by `Caedmon`, `Tullysaurus`, or `theron17` before merging into `main`.
  - Prefer the merge methodology chosen by the project (Squash+merge or Merge commit). If unsure, ask the reviewers.

## Issues (bug reports and feature requests)

When opening an issue, please include:

- A clear title.
- Reproduction steps (minimal if possible): exact steps, inputs, expected behavior, actual behavior.
- Environment details: OS, Java version, Gradle version, WPILib version, simulator vs real robot.
- Relevant logs, stack traces, screenshots, or a minimal sample project that reproduces the problem.
- Labels/priority: add labels like `bug`, `enhancement`, `documentation` when appropriate.

If you found a bug and can prepare a fix, open a PR and reference the issue with `Fixes #<issue-number>` to auto-close it when merged.

## Tests

- Add unit tests for new logic where reasonable. Keep tests fast and deterministic.
- For robot/simulation code, prefer contract tests or small integration tests run in CI.

## Small examples and snippets

Follow examples already present in the repo (see `src/main/java/frc/robot`) for project-specific conventions, wiring, and generated constants. Keep code consistent with existing patterns.

## Helpful practices

- Keep changes small and focused — easier to review.
- Run `./gradlew build` before opening a PR to catch compile errors and test failures.
- Consider adding a format/lint pre-commit hook in your IDE to auto-fix style issues before committing.

---

Thank you for contributing — your time and care make this project better.