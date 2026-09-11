# Claim-bounded social demo media — 2026-08-16

> Status: **PUBLICATION_CANDIDATE / NOT_PUBLISHED**
>
> External publication authorized: **false**
>
> Remote writes performed: **none**

## Finding

The curated release bundle still required a social post, PNG card, and MP4
created for `v0.2.2`. The video generator burned `v0.2.2`, the retired
`run_autoware_map_beginner.sh` command, three numerical benchmark statements,
and the old release URL into a new render. The card generator independently
used the older `run_autoware_quickstart.sh` route and optional local benchmark
files whose absence became `n/a`. Re-running either tool could therefore put
stale commands or unbound performance copy into a future release bundle.

The exact previous card, video, and post SHA-256 values were respectively
`137ba04b3e1a74e2d2ff988b3feeb9de9de088c5affa7d324df35e7aba4f175a`,
`59aa6ac10cb21fa8c9b6ecd4cf106b086ed555bd7674c9a38c6bebff178eb3b4`,
and
`073ef77056355eb560b6702264790bbe0b3a7108a9ca7cf07f39ac4ef4e391e7`.
They remain recoverable from history but are no longer the active release
media path.

## Local repair

Exact generator implementation
`d0c84bb9bb7bef37d7e318000e3071a7f536d631` adds one Draft 7 contract for
all slide copy, canonical commands, source-image paths, and source-image
SHA-256 values. Both the card and video generator consume that contract.

The contract and generators now:

- require product `VERSION` and canonical README/Getting Started commands to
  agree before rendering;
- reject the retired versions, commands, release language, and numerical
  benchmark phrases found in the old media path;
- bind all three source images by path and SHA-256;
- produce four bounded slides with no numerical performance, release,
  package-availability, or sensor-compatibility claim;
- emit a non-overlapping four-cue English WebVTT file and concise Japanese and
  English copy that remains explicitly `NOT_PUBLISHED`;
- require an exact clean `HEAD` for `PUBLICATION_CANDIDATE`, while dirty output
  is marked `LOCAL_PREVIEW` and cannot be mistaken for publishable media;
- generate the MP4, captions, post, and schema-valid SHA-256 manifest in a
  private temporary directory before one local directory rename; and
- refuse overwrite, symlink input/output, unbound images, command drift,
  contract drift, invalid H.264 geometry/frame rate, or a duration at or above
  three minutes.

The release inventory now carries the generators, current post, current card,
MP4, WebVTT, manifest, contracts, and schemas. It no longer carries the old
`v0.2.2` post.

## Exact generated candidate

The tracked media was generated from the clean exact implementation revision
above. Its manifest reports `PUBLICATION_CANDIDATE`, but always keeps external
publication authority and remote writes false.

| Artifact | Size | SHA-256 |
| --- | ---: | --- |
| `lidarslam/images/social_autoware_map_authoring.png` | 672,167 bytes | `6b31035ad40f6d8e567bc3a1f62efdb4cb470d26d8a49bb198f2f3c15ceed4c9` |
| `lidarslam/images/social_autoware_map_authoring_demo.mp4` | 447,994 bytes | `57bb4d1ead298529ddd4323319c16180d390833e1fa8a6bfe592eab4db82e194` |
| `lidarslam/images/social_autoware_map_authoring_demo.en.vtt` | 562 bytes | `18b14ad549fe6f7e0d3c28f81ee7643c318316c0a2eda2bd30a663a0c65d02b7` |
| `lidarslam/images/social_autoware_map_authoring_demo.manifest.json` | 2,419 bytes | `e122d89d272e4cc0ce7d48a88fc6a29d781e460ca337f28d23649699ccf5847c` |
| `docs/social/autoware_map_authoring_post_v0.9.1.md` | 1,953 bytes | `e44aa1b38aabac8812a00ba0f284e0a87e08c0adb612433b37e6ff4e8f3eaa29` |

`ffprobe` reports one H.264 stream, 1280 x 720, 24 fps, and 10.666667 seconds.
The four representative frames were inspected after generation; the original
benchmark-labelled second background was rejected and replaced with a
hash-bound, text-free trajectory image before the candidate was sealed.

## Validation and boundary

- social media contract/generator regressions: **11 passed**;
- public docs/release entrypoint regressions: **25 passed**;
- exact changed-file Jazzy `ament_flake8`: **PASS**;
- Python byte-compilation and `git diff --check`: **PASS**;
- strict MkDocs after final release-inventory synchronization: **PASS**.

This creates no release, upload, announcement, post, tag, image publication,
or community mutation. The exact-version link points to the clean generator
revision and becomes a public candidate source only when its carrying product
branch is pushed. A maintainer must still re-run the publication audits and
choose E4/E3 scope separately before posting any generated copy or media.
