# Security Policy

## Supported versions

Security fixes are made against `develop` and the latest tagged `0.x` release.
Older release lines may be asked to upgrade before a fix is provided.

| Version | Security support |
| --- | --- |
| `develop` | Active |
| latest tagged `0.x` release | Active |
| older releases | Best effort |

## Reporting a vulnerability

Do not open a public issue for a suspected vulnerability.

Report it privately to `rsasaki0109@gmail.com` with the subject
`[lidarslam_ros2 security]`. Include:

- affected version, commit or image digest;
- operating system and ROS distribution;
- the smallest reproduction available;
- expected impact and whether physical robot operation is involved;
- whether the report or proof of concept may be shared publicly.

The maintainer will acknowledge receipt when available, assess scope, and
coordinate disclosure and a fix. This is a best-effort community project and
does not promise a contractual response time.

## Security scope

In-scope examples include:

- command or path injection through documented product inputs;
- unsafe archive extraction or download verification;
- release, container or dependency supply-chain compromise;
- unintended disclosure of local files or credentials;
- remotely triggerable denial of service in a supported product path.

General mapping accuracy, a poorly calibrated sensor, unsupported hardware and
research-algorithm quality normally belong in the public issue tracker unless
they create a concrete security or safety impact.

## Robotics safety

This software authors maps; it is not safety-certified localization or
autonomous-driving software. Treat generated maps as untrusted until they pass
the bundled verifier and an operator reviews them. A verifier `PASS` checks
the documented file contract, not the safety of deploying the map on a robot.
