# Security Policy

## Supported versions

OpenRoad Dynamics is pre-release research software. Only the `main` branch receives fixes.

## Reporting a vulnerability

Please **do not** open a public issue for security problems. Use GitHub's private
vulnerability reporting ("Security" tab → "Report a vulnerability") on this repository.

We aim to acknowledge reports within 7 days. Relevant classes of issues include:

- Memory-safety bugs in the physics core (out-of-bounds access, use-after-free) that can be
  triggered by crafted vehicle definition files or network-replicated state.
- Denial of service via malformed data files (e.g. NaN/Inf propagation, unbounded loops,
  excessive allocation).
- Multiplayer state-sync issues that allow a client to inject physically impossible state
  into an authoritative simulation.

## Scope notes

- Vehicle definition files and telemetry inputs must be treated as untrusted. Parsers must
  validate ranges and reject non-finite values.
- The project does not operate any online services.
