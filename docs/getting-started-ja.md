# 日本語クイックスタート

`lidarslam_ros2`は、rosbag2からAutowareで読み込める地図一式を作成します。
初めての場合は、手元の環境に合う入口を1つだけ選んでください。

## まず選ぶ

| 手元にあるもの | 最初のコマンド |
| --- | --- |
| インストール済みだが、状態が分からない | `lidarslam-map doctor` |
| Dockerだけで固定デモを試したい | [Dockerで固定デモ](#docker-demo) |
| sourceから構築したい | `bash scripts/source_quickstart.sh` |
| 対応する自分のrosbag2がある | `lidarslam-map start /path/to/rosbag2` |
| bagのtopicや互換性が分からない | `lidarslam-map doctor /path/to/rosbag2` |

インストール後に端末で引数なしの`lidarslam-map`を実行すると、インストール確認、
固定デモ、自分のbag、過去のsessionから安全な入口を選べます。

### 日本語のDockerとsource経路を選ぶ

最初のfirst-mapでは、Dockerまたはsourceのどちらか1つだけを選びます。DockerはROS 2の
workspaceを準備せずに公開済み固定デモを試すcontrol experiment、sourceはHumble/Jazzyの
環境でhelperを確認しながら構築する経路です。出力directoryやsessionを別経路で使い回さず、
経路を変える場合は新しいoutputを使ってidentityを記録します。

| 経路 | こういう場合 | 最初の1コマンド | ここで止めて確認 |
| --- | --- | --- | --- |
| Docker fixed first-map | ROS 2 workspaceなしで、公開済みの基準デモを先に試したい | 下記のDocker route command | 完了後に`map_verify: PASS`、receiptの`status: PASS`、`--version`のidentityを確認する。PASSでなければ同じoutputをsourceで再利用しない |
| source quickstart | Ubuntu 22.04/HumbleまたはUbuntu 24.04/Jazzyで、sourceから構築する | `bash scripts/source_quickstart.sh --dry-run` | `dry_run`のplanと`run.command_shell`をreviewしてから実行へ進む。`--version`のrevisionを記録し、Dockerのreceiptと混ぜない |

Docker routeの最初のcommand（Humble）は次の1つです。Jazzyではimage tagだけを
`v0.9.0-jazzy`に置き換えます。

```bash
docker run --rm \
  -e LIDARSLAM_HOST_UID="$(id -u)" \
  -e LIDARSLAM_HOST_GID="$(id -g)" \
  -v "$PWD/lidarslam_output:/lidarslam_ws/output" \
  ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble
```

source routeの最初のcommandは、書き込み前にplanだけを確認する次の1つです。

```bash
bash scripts/source_quickstart.sh --dry-run
```

Dockerのidentityは公開済み安定版`v0.9.0-humble`または`v0.9.0-jazzy`です。source helperは
まだ公開・tag付けされていない`v0.9.1`候補を扱うため、sourceの結果を公開releaseやDocker
identityとして報告しません。GLIMのようなPPA/package-manager経路は未対応なので、経路選択で
これらを代替手段として追加しません。最初のcontrol experimentが失敗した場合は、表示された
`Details:`と`Next:`を読み、経路やoutputを手で混ぜずに保存済みの指示へ戻ります。

### 日本語の経路切替とfresh output復旧

Dockerまたはsourceのfirst-mapが途中で終わった後に経路を変える場合は、元のrunを別経路で
続けません。まず元のlocal-onlyの`map_session_recovery.json`、`Details:`、`Next:`を読み、
次の3つを区別します。

| 状態 | 進め方 | 証跡の境界 |
| --- | --- | --- |
| 同じsessionで`resume.available: true` | 保存された`next_command`の`--resume`を編集せずに実行する | post-processingだけを再開し、経路変更やmapping再実行をしない |
| 同じpinned setupで`retry.available: true` | 保存された`retry.command`を編集せずに実行する | `retry.output_dir`の新しいoutputだけを使い、元のmap・receipt・manifestを上書きしない |
| Docker/sourceの経路を変える、またはresume/retryがない | doctorと新しい経路のfirst commandを確認し、fresh outputで新しいrunを開始する | 古いmap、session、receipt、manifestを新runへコピー・再利用せず、identityもrunごとに記録する |

`--resume`は既存sessionの安全なterminal post-processing用であり、Dockerからsourceへ、または
sourceからDockerへ切り替えるcommandではありません。保存済み`retry.command`も元のpinned setupを
再試行するためのものなので、経路を変えるときは使わず、新しい経路の案内へ戻ります。変更後は
たとえば`output.docker`と`output.source`のように新しいoutputを選び、`v0.9.0` Dockerまたは
`v0.9.1` source候補のidentityをそれぞれ`--version`など実際の出力から記録します。

元の証跡は削除・uploadせず、旧runと新runのreceiptやhashを混ぜません。新しいsessionで
`report`が`READY FOR REVIEW`を返した場合だけ、その新runのreceiptを確認します。
viewerの見た目からcommandを再構成したり、古いreceiptを新しいmapへコピーしたりせず、支援が
必要ならlocal pathを含むrecovery JSONではなく、後述のprivacy-bounded support reportだけを使います。

## 1. インストールを確認する

```bash
lidarslam-map doctor
```

この確認はネットワークへ接続せず、ファイルも書きません。ROS 2、bag reader、
製品ファイル、デモ用の空き容量を確認します。不足が複数あっても、依存順で選んだ
`Do this now`を1つだけ先頭に表示し、残りは後続checkとして残します。その操作後に
`doctor`を再実行すると、次のblockerが1つ選ばれます。JSONは全finding固有の復旧操作を
保持し、同じ選択をtop-levelの`next_action`でも返します。自分のbagを先に確認する場合は
次を実行します。

```bash
lidarslam-map doctor /path/to/rosbag2
```

これはtopic、PointCloud2のfield、timestamp順、利用可能なprofileを確認します。

### JSON診断でreason-codeとNext-actionを読む

自分のbagをmappingへ渡す前に、診断結果を機械可読な形でも確認できます。
このコマンドはbagをread-onlyで読み、ネットワークへ接続せず、出力ファイルも作りません。

```bash
lidarslam-map doctor /path/to/rosbag2 --json
```

doctorのbag reportでは`findings[].code`を安定したキーとして使います。`start`の
dry-run JSONや保存済みsessionのrecovery JSONでは、全体の`reason.code`と各項目の
`findings[].code`を使います。`message`、`Details:`、viewerで見えた症状、英語の説明文を
手がかりにコードを推測したり、automationのキーにしたりしません。

診断結果の`next_action`、`Next:`、`next_command`は保持された次の操作です。自分で
pathやoptionを組み立て直さず、そのcommandを確認してからそのまま実行します。JSONには
bagのlocal pathやtopic名が含まれる場合があるため、raw JSONをissueへ貼り付けません。
支援が必要なときは、後述のサニタイズ済みsupport reportだけを内容確認後に使います。

## 2. Dockerで固定デモ {#docker-demo}

ROS 2 workspaceを構築せず、固定MID-360デモから検証済み地図を作ります。

```bash
docker run --rm \
  -e LIDARSLAM_HOST_UID="$(id -u)" \
  -e LIDARSLAM_HOST_GID="$(id -g)" \
  -v "$PWD/lidarslam_output:/lidarslam_ws/output" \
  ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble
```

このコマンドは公開済み安定版`v0.9.0-humble`に固定しています。`develop`の
移動タグを使わないため、初回導線の内容が不意に変わりません。`v0.9.1`は
まだ公開・tag付けされていない候補版なので、候補版を試す場合はsource helperを
使ってください。Ubuntu 24.04/Jazzyでは
`ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-jazzy`を使います。初回は517 MBの公開bagを
取得し、最低8 GiBの空き容量を使います。目安は約30分ですが、回線とCPUで
変わります。出力先は`lidarslam_output/mid360_demo/`です。

公開済み`v0.9.0` imageには、現在の`lidarslam-map report` handoff commandがまだ
ありません。固定Dockerデモが完了したら、同じoutputの
`first_map_validation_receipt.json`を開いて`status: PASS`と全checkを確認し、receiptだけを
独立validationのissue formへ添付します。sourceまたは`report`対応済みimageでは、後述の
`lidarslam-map report`でreceipt-bound handoffを作成します。

summaryだけを再表示したい場合は、公開済みstable imageの次のread-only fallbackも使えます。
outputはread-only mountし、networkも無効にしています。

```bash
docker run --rm --network none \
  -v "$PWD/lidarslam_output:/output:ro" \
  ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble \
  bash -lc 'helper="$(ros2 pkg prefix lidarslam)/share/lidarslam/product/scripts/create_first_map_validation_receipt.py"; python3 "$helper" /output/mid360_demo'
```

Ubuntu 24.04/Jazzyではimage tagを`v0.9.0-jazzy`へ置き換えます。表示されたPASS summaryを
reviewし、すでに生成されたJSON receiptだけを添付します。
pull後にimmutable image identityも確認します。

```bash
docker image inspect \
  --format='{{index .RepoDigests 0}}' \
  ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble
```

`ghcr.io/...@sha256:...`が表示されない場合はidentity未検証として停止し、mutable tagを
代用しません。

```bash
lidarslam-map report /path/to/session-or-demo-output --json
```

固定デモのoutputは`session.json`ではなくreceiptを直接持つため、
`/path/to/session-or-demo-output`には`lidarslam_output/mid360_demo/`を指定します。

## 3. sourceから構築する

Ubuntu 22.04/HumbleまたはUbuntu 24.04/JazzyにROS 2を導入してから実行します。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive https://github.com/rsasaki0109/lidar_slam_ros2.git
cd lidar_slam_ros2
bash scripts/source_quickstart.sh
```

helperはHumble/Jazzyを検出し、このrepositoryの6 packageだけを準備・buildして、
固定デモまで実行します。実行内容だけを先に確認する場合は次を使います。

```bash
bash scripts/source_quickstart.sh --dry-run
```

CIやwrapperから機械的に確認する場合は`--json`を追加します。

```bash
bash scripts/source_quickstart.sh --dry-run --json
```

これはversion付きの[`source-quickstart-plan-v1` schema](schemas/source-quickstart-plan-v1.schema.json)
をstdoutだけに出します。network、APT、submodule checkout、build、demo、filesystemへの
writeは行わず、不足しているbootstrap actionと実行予定のcommand arrayを返します。local pathを
含むため、raw JSONはissueへ貼らず手元で扱います。

buildだけなら`--build-only`、画面を開かない環境なら`--viewer none`を追加します。
完了時に表示される絶対パスの`lidarslam-map`は、新しい端末でも対応するworkspaceを
自動で有効化します。

固定デモを実行する前に計画を保存して確認する場合は、次を使います。

```bash
lidarslam-map demo ~/ros2_ws \
  --viewer none --dry-run --json \
  --output /tmp/mid360-demo-plan.json
```

`--output`は計画JSONまたは人間向けカードを一度だけ作成し、既存ファイルを上書きしません。
これはread-onlyの確認用で、bagのdownload、mapping、公開操作は行いません。

!!! note "現在の配布境界"
    GLIMのようなPPA/package-managerの導入経路は、依存packageのrosdistro審査が
    完了するまで未対応です。現時点の正規経路はDockerまたは上記source helperです。

### versionとsupport境界を記録する

supportや独立validationへ進む前に、実際に使ったCLIのversionとrevisionを記録します。
これはネットワークへ接続せず、mapやsessionを書き込みません。

```bash
lidarslam-map --version
```

Dockerの固定デモは公開済み安定版`v0.9.0-humble`または`v0.9.0-jazzy`を使います。
`v0.9.1`はまだ公開・tag付けされていないsource候補なので、source helperで実行した
場合はcandidateとしてそのversion/revisionを報告します。`develop`の移動tagを使ったり、
出力にないrelease identityを推測したりせず、`--version`の出力をそのままsupport report
やvalidation formへ転記してください。

## 4. 自分のbagを地図にする

インストール済みの場合は、`metadata.yaml`を含むrosbag2 directoryを渡します。

```bash
lidarslam-map start /path/to/rosbag2
```

`start`は書き込み前にsensor、topic、timestamp、profile、calibrationを説明し、
確認後にmapping、地図検証、offline reviewまで進みます。長時間処理の前に確認だけ
行うには次を使います。

```bash
lidarslam-map start /path/to/rosbag2 --dry-run
```

### Ouster、Velodyne、RoboSenseなど別のPointCloud2 LiDARを使う

最初にこのpackageのlaunch fileやYAMLをfork・編集しないでください。実際のsensorや
simulatorからrosbag2を記録し、先に
`lidarslam-map doctor /path/to/rosbag2`、続いて上の`start`を実行します。
`doctor`は記録済みtopicの型、`header.frame_id`、point field、timestamp順、利用できる
maintained profileを確認します。`start`はその選択を保持し、書き込みやmapping開始前に
calibrationのreviewを求めます。

安全なmaintained pathがない場合は、mapping前に安定したreason codeとcopy-readyな
次の1コマンドを表示して停止します。PointCloud2を検出できることは、そのvendorや
hardwareの検証済み対応、または精度保証を意味しません。

### 自分のbagを先にdry-runで確認する

長時間のmappingやファイル作成へ進む前に、選択されるprofile、topic、frame、
calibration、実行予定のcommandをJSONで確認できます。

```bash
lidarslam-map start /path/to/rosbag2 \
  --yes \
  --dry-run \
  --json
```

`--dry-run`ではsession bundle、`sensor_setup.json`、map outputを残さず、mappingも
開始しません。`--yes`はこの確認を非対話で実行するための指定であり、dry-runを外した
実行の書き込みを許可するものではありません。readyな結果ではJSONの`status`が
`dry_run`になり、`run.command_shell`に表示された保持済みの実行commandを確認できます。

安全なprofileを選べない場合は終了コード2になり、`reason.code`、`findings[].code`、
各findingの`next_action`、`next_command`が診断を示します。sessionやmapはこの場合も
作成されません。`next_command`がdoctorを示すときはそのcommandへ戻り、viewerの症状から
mapping commandを組み立て直しません。

確認したplanで実際にmappingへ進むときは`--dry-run`を外し、端末では表示内容を確認して
から開始します。ヘッドレス環境ではbrowserを起動せず成果物を保持するため、明示的に
`--viewer none`を使います。

```bash
lidarslam-map start /path/to/rosbag2 --viewer none
```

非対話で実行する場合も、planをレビューした後だけ`--yes --viewer none`を追加します。
JSONや表示されたcommandにはlocal pathが含まれる場合があるため、確認結果はlocal-onlyで
扱い、raw outputをissueへ貼り付けません。

ROS 2をhostへインストールせず、checkout済みrepositoryからDockerを使う場合は
次の1コマンドです。bagはread-onlyでmountされます。

```bash
bash scripts/docker_map_bag.sh /absolute/path/to/rosbag2
```

実行前の確認結果をCIやwrapperから機械的に読む場合は、`--dry-run`と`--json`を指定します。

```bash
bash scripts/docker_map_bag.sh --dry-run --json /absolute/path/to/rosbag2
```

出力はversion付きの[`docker-map-bag-plan-v1` schema](schemas/docker-map-bag-plan-v1.schema.json)
だけをstdoutへ返します。Docker呼び出し、network access、filesystem write、output directoryの
作成は行いません。inputはread-only mountとして記録され、image identityとcontract preflightは
実際のrunまで保留されます。JSONにはlocal pathが含まれるため、raw outputはissueへ貼らずlocal-only
で扱います。

## 成功と失敗の見分け方

成功時は`map_verify: PASS`とともに、次の場所が1画面に表示されます。

- Autoware用`pointcloud_map/`
- `map_projector_info.yaml`
- 自動生成された`lanelet2_map.osm`
- 検証receiptとoffline review

失敗時は`[reason-code]`、`Details:`、`Next:`を探してください。保持された次の操作が
安全なterminal post-processingだけを再開する`--resume`なら、表示されたcommandを
そのまま実行します。

```bash
lidarslam-map demo /path/to/work_dir --resume
```

`--resume`はmappingを再実行せず、安全なterminal post-processingだけを再開します。
bagのtopic、field、timestamp、profileが不明、または診断が入力修復を求めている場合は、
mappingやviewerを再試行せず`lidarslam-map doctor /path/to/rosbag2 --json`へ戻ってください。
`reason.code`や`findings[].code`の`Next:`が新しいoutput directoryでのretryを示す場合だけ、
保持されたcommandを使い、失敗したrunを上書きしません。

### 検証済みmapと表示できるmapを区別する

viewerで`pointcloud_map/`やpreviewが開けても、それだけではtrusted resultではありません。
表示できることはmap outputが存在することを示すだけで、Autoware verificationや証跡の整合性が
確認されたことは示しません。trusted resultとして扱えるのは、同じrunで`map_verify: PASS`が
出て、保存された`first_map_validation_receipt.json`の`status: PASS`（receipt内の全checkもPASS）を
確認できる場合だけです。

| 表示・receiptの状態 | 意味 | 次の操作 |
| --- | --- | --- |
| `map_verify: PASS` とreceiptの`status: PASS` | map、verification、manifestに結び付いた証跡が揃ったtrusted result | versionとreceiptを確認してから、必要ならsupportまたは独立validationへ進む |
| viewerだけ表示される、または`NOT VERIFIED` | map outputはあってもverificationを実行していない・完了していない | trusted evidenceとして共有せず、保存されたdiagnosisと`inspect` commandを先に読む |
| `UNAVAILABLE`、receiptの欠落・不正・`FAIL` | receiptから検証結果を確定できない | viewerの見た目からPASSを推測せず、同じsessionの証跡をinspectする |

`NOT VERIFIED`は「mapが失敗した」という推測ではなく、verificationが未実行または未完了という
境界です。`UNAVAILABLE`はreceiptがない、壊れている、または必要な証跡が揃っていない状態で、
近くにあるmapやpreviewからPASSを補いません。まず同じoutput/sessionの
`first_map_validation_receipt.json`を開き、`status`、`verification`、checkの結果を読みます。
別sessionからコピーしたreceiptを現在のmapの証拠には使いません。

`status`がPASSでない場合は、supportや独立validatorへ渡す前に、保存された
`autoware_map_diagnosis.md`/JSON、`verify_autoware_map.log`、`run_manifest.json`を読み、保持された
`Details:`と`Next:`に従います。必要なら次のread-only診断を実行します。

```bash
lidarslam-map inspect /path/to/output --write
```

`inspect`の結果が示す原因を直して同じrunを再確認するまで、表示されたmapをverified resultとして
扱いません。独立validationへ進む場合も、内容をreviewしたPASS receiptだけを使い、map、bag、
raw log、preview、local pathを添付しません。

### 実行は終わったがmapの見た目がおかしい場合

launch fileやYAMLを推測で変更せず、元のsessionを残して、見えた症状を1つだけ既存の
`inspect`へ渡します。

```bash
lidarslam-map inspect /path/to/session_bundle \
  --bag /path/to/rosbag2 \
  --symptom map-spins-or-spirals
```

指定できるcodeは`map-spins-or-spirals`、`pose-drifts-or-oscillates`、
`map-stops-early`、`map-is-too-sparse`、`map-is-not-visible`です。入力topic・時刻、
物理calibration、TF、runtime完了、map保存、viewerの順に確認し、保持済みpathを使った
`doctor`、`inspect`、`view`、`support`だけを次のcommandとして返します。cardをsessionへ
保存する場合は`--write`、local automationでは`--json`を追加します。

これはユーザーが申告した症状の整理であり、原因の自動判定や精度結果ではありません。
parameterを自動変更せず、mappingを再実行せず、support bundleをuploadしません。見た目が
変わっただけでsensor対応や精度改善を主張せず、比較する場合は旧runを上書きしないfresh
outputを使います。raw diagnosis JSONにはlocal pathが含まれ得るためissueへ貼りません。
`--write`で保持した症状は、`support`では固定codeと「ユーザー申告」の境界だけが
sanitized reportとissue bodyへ引き継がれます。title、check、command、自由記述はlocalに
残り、root causeの自動判定として共有されません。

### receiptのsessionとstatusを確認する

`first_map_validation_receipt.json`だけを別のmap directoryへコピーしても、そのmapの証拠には
なりません。receiptを読むときは、同じsessionの`session.json`にある
`artifacts.validation_receipt`のpathと`map_output`を基準にし、receiptの`run.run_id`が
同じ`run_manifest.json`の`run_id`と一致することを確認します。receiptの
`verification.manifest_sha256`は、そのsessionの`run_manifest.json`に結び付いた値です。
別runのreceipt、古いsessionのreceipt、名前だけ変更したreceiptを混ぜません。

sessionを一覧から選ぶ場合は、local-onlyのJSONを確認します。

```bash
lidarslam-map sessions ./output --json
```

独立validationやsupportへ進む前の最終ゲートは、元のsession bundleに対する次のread-only
再検証です。

```bash
lidarslam-map report /path/to/session_bundle
```

固定Docker/source demoは`session.json`を作らずreceipt-bound outputだけを残すため、その場合は
`lidarslam-map report /path/to/output/mid360_demo --json`のようにoutput directoryを渡します。
どちらもmanifest、diagnosis、verification logのhashを再検証するread-only handoffです。

公開source checkoutが`report <output>`対応前の場合は、checkout内のhelperでreceiptだけを
read-only再生成・表示できます。

```bash
python3 scripts/create_first_map_validation_receipt.py \
  ~/ros2_ws/output/mid360_demo --json
```

`PASS`のJSON receiptだけをreviewして添付し、`--write`は使いません。map、manifest、diagnosis、
logは添付しません。

このcommandは、sessionがverifiedであること、receiptがそのsession内の通常ファイルであること、
receiptのschemaと全check、manifest・diagnosis・verification logのhashが一致することを確認します。
`READY FOR REVIEW`が出た場合だけ、表示されたreceipt pathのJSONを内容確認して共有候補にします。
自動化では`report --json`を使えます。この再検証は書き込みもGitHubへの通信も行いません。
旧表記の`support /path/to/session_bundle --first-map`も同じ検証として利用できます。

`status: PASS`のreceiptがあっても、再検証がreceipt mismatch、missing、invalid、またはnot PASSで
終了した場合は、その証拠をtrustedとして使いません。receipt、`run_manifest.json`、`session.json`を
手編集してhashを合わせたり、古いreceiptを新しいmapへコピーしたりせず、保存された`Details:`、
`Next:`、`retry.command`またはverification-enabledな新しいoutput commandへ戻ります。元の証跡は
削除・uploadせず、support reportと公開添付のprivacy境界を守ります。

### first-map検証パッケージの事前監査

source checkoutからDockerまたはsourceの経路を選ぶ前に、次の監査を実行します。

```bash
python3 scripts/check_first_map_verification_package.py --json
```

この監査はlocal-onlyかつread-onlyです。`READY`のときだけ先へ進みます。Docker/sourceのhandoff、
receipt schema、runtime互換性guard、公開ページのmarker、CI gate、release bundle inventoryが
一つの検証パッケージとして揃っていることを確認します。

### receipt再検証に失敗したときの復旧

`report`がrejectしても、元のmap、session、receipt、manifestは削除されません。
これは現在のevidenceがtrusted resultとして使えないという判定であり、mapが消えたという意味では
ありません。旧outputを削除せず、receipt、`run_manifest.json`、`session.json`の内容やhashを手で
書き換えて再検証を通そうとしません。

まずlocal-onlyのsessionと保存された復旧情報を確認します。

```bash
lidarslam-map sessions ./output --status action_required --viewer none --json
lidarslam-map inspect /path/to/output --write
```

`map_session_recovery.json`、diagnosis、`Details:`、`Next:`を順に読みます。`resume.available`
がtrueで`next_command`が`--resume`なら、表示されたcommandをそのまま実行し、mappingを再実行
しません。post-processingだけが完了すれば、同じsessionのreceiptをもう一度確認できます。

`retry.available`がtrueなら、保存された`retry.command`を編集せずに実行します。これはpinned setup
を保持したまま、通常は`map.retry`または`map.retry-2`のような新しいoutputへ書きます。旧sessionの
証跡と新しいretryの結果を混ぜず、新retryで新しく生成されたreceiptだけを再検証します。

resumeもretryも利用できない場合、または元のrunがverification offの診断だった場合は、doctorの
結果を確認してから、古いoutputとは別のpathにverificationをrequiredにした新しいrunを開始します。

```bash
lidarslam-map start /path/to/rosbag2 \
  --output-dir /path/to/output.verified \
  --verification required
```

viewerの見た目からcommandを再構成したり、古いreceiptを新しいmapにコピーしたりしません。新しい
sessionで`report`が`READY FOR REVIEW`を返し、receiptを内容確認できた場合だけ、
supportまたは独立validationの候補にします。旧証跡、bag、map、raw logは削除・uploadせず、公開時は
review済みreceiptだけを使います。

### 失敗したrunを上書きせず再試行する

失敗したrunをもう一度mappingするときも、元のdirectoryを削除したり同じpathを指定したり
しません。`start`の`--output-dir`がすでに存在する場合は`output directory already exists`
で停止し、既存のsessionやmapを上書きしません。

保存された`map_session_recovery.json`では、次の3つを分けて読みます。

- `setup_bundle`: `sensor_setup.json`、pinned parameter、session pageなどの保持されたsetup。
- `evidence`と`files_preserved: true`: 元のrunで得られたmanifest、diagnosis、log、receiptの証跡。
- `retry.available: true`: `retry.command`が同じpinned setupを使い、`retry.output_dir`
  （通常は`map.retry`のような新しいdirectory）だけへ出力する再試行。

原因を直した後は、JSONに保持された`retry.command`をpathやoptionを編集せず、そのまま
実行します。viewerの見た目から新しいmapping commandを作りません。`resume.available: true`
で`next_command`が`--resume`なら、まずpost-processingだけを再開し、mappingを再実行しません。
retryが無い場合に新しい`start`を意図して行うときだけ、古いsessionとは別の新しい
`--output-dir`を指定し、先にdoctorの診断を確認します。

recovery JSONとcommandにはbagやlocal pathが含まれるため、これらはlocal-onlyで扱います。
元のsession、map、raw logを削除・uploadせず、支援が必要な場合は後述のサニタイズ済み
support reportだけを確認して共有します。

### mapまたはviewerが空のとき: 3つの確認

初回は自分のbagやframeを変更する前に、固定公開デモをviewerなしで実行して
基準結果を確認します。

```bash
lidarslam-map demo ~/ros2_ws --viewer none
```

live入力を確認する場合は、まずtopicの型を確認します。

```bash
ros2 topic list -t
```

出力のうち`[sensor_msgs/msg/PointCloud2]`と表示された行のtopic名を選び、
`<POINTCLOUD_TOPIC>`をその名前に置き換えます。例えば
`/points [sensor_msgs/msg/PointCloud2]`なら、コマンドには`/points`だけを入れます。
PointCloud2の行がない場合は、publisherまたはlaunchのremapを直してから再確認します。
山括弧を残したまま実行せず、次の順に確認します。

1. **PointCloud2が届いているか**

   ```bash
   timeout 5s ros2 topic hz --window 5 <POINTCLOUD_TOPIC>
   ```

   `average rate:`が正の値で続けば入力は到達しています。0またはpublisherなしなら、
   topicのremapまたはpublisherを直してから再確認します。

2. **`frame_id`が空でないか**

   ```bash
   timeout 5s ros2 topic echo --once --field header.frame_id <POINTCLOUD_TOPIC>
   ```

   出力されたframe名を次のTF確認にそのまま使います。出力が空、または5秒で
   timeoutした場合は、publisherの`header.frame_id`を修正してから再確認します。
   viewerのframe名を推測して先に進めません。

3. **TFがつながっているか**

   check 2で出た空でないframe名を`<POINTCLOUD_FRAME>`に入れます。
   `<TF_TARGET_FRAME>`にはruntimeまたはviewerが基準にするtarget frameを入れます。
   例えばcheck 2の出力が`livox_frame`なら、`<POINTCLOUD_FRAME>`だけを
   `livox_frame`に置き換えます。viewerでframe名を推測したり、山括弧を残したまま
   実行したりしません。

   ```bash
   ros2 run tf2_ros tf2_echo <TF_TARGET_FRAME> <POINTCLOUD_FRAME>
   ```

   `At time ...`が繰り返し表示されることを確認します。利用できない場合は、
   この2つのframeのsource/targetの向きとstatic extrinsicを直してから、同じ実際の
   frame名で再確認します。

3つが通ってもmap messageがない場合は、viewer設定を変える前に次を確認します。

```bash
timeout 5s ros2 topic echo --once /map/pointcloud_map
lidarslam-map inspect /path/to/output --write
```

messageがなければ生成された`autoware_map_diagnosis.md`の最初のfindingに従います。
messageがあればmapは存在するため、viewerのfixed frameを`map`、topicを
`/map/pointcloud_map`に設定し、offline previewを確認します。bag、map、raw logを
uploadする必要はありません。

### ブラウザが開かない・ヘッドレス環境の場合

offline previewはネットワーク不要のself-contained HTMLです。ブラウザが自動で
開かない場合やヘッドレス環境では、HTMLの生成とブラウザを開く操作を分けます。
`--no-open`を付けるとブラウザを起動せず、コマンドが表示する`HTML:`の絶対パスを
使えます。

```bash
lidarslam-map view /path/to/output \
  --viewer browser \
  --no-open
```

出力例の`HTML: /path/to/output/preview/mid360_robot_3d_map_preview.html`にある
実際の絶対パスを確認します。デスクトップ環境で開く場合はそのHTMLをブラウザで
開き、ヘッドレス環境で生成した場合は管理下の方法でHTMLだけをデスクトップ
環境へコピーして開きます。previewの保存先を明示したい場合は`--preview-dir`を
追加します。

```bash
lidarslam-map view /path/to/output \
  --viewer browser \
  --no-open \
  --preview-dir /path/to/preview
```

`--preview-dir`は`--viewer browser`と一緒に使います。生成されたHTMLにはmapの
表示データが含まれるため、bag、map、raw logと同じく非公開の成果物として扱い、
GitHub issueへuploadしません。共有が必要な場合は、先にサニタイズ済みのsupport
reportを確認します。

### 保存したsessionを探して復旧する

前回の実行結果をtimestamp付きdirectoryから探す代わりに、session履歴を使います。
通常は`./output`直下の保存済みbundleだけを検査し、ローカルのcatalogを作ります。

```bash
lidarslam-map sessions
```

途中で止まったsessionだけを、ブラウザを開かずに確認する場合は次を使います。

```bash
lidarslam-map sessions ./output \
  --status action_required \
  --viewer none
```

catalogや端末出力に`Details:`と`Next:`が表示されたら、保存された診断結果を読み、
`Next:`の行に表示された実際のcommandをそのまま実行します。viewerを変更する前に
`map_verify`、`autoware_map_diagnosis.md`、TF・topicのfindingを確認してください。
自動処理や確認だけなら、catalogを開かずJSONを読むこともできます。

```bash
lidarslam-map sessions ./output \
  --status action_required \
  --viewer none \
  --json
```

session履歴、catalog、診断、previewはlocal-onlyです。bag、map、raw log、preview
HTMLをGitHub issueへuploadせず、支援が必要な場合はサニタイズ済みsupport report
だけを内容確認後に共有します。

### 支援へ共有する前に確認する

まずsupport reportをJSONで確認します。これはread-onlyで、ZIPを作成せず、GitHubへ
送信もしません。

```bash
lidarslam-map support /path/to/session_bundle --json
```

ZIPが必要な場合はsession bundleの隣に作成される次の3ファイルだけを確認します。
`README.txt`、`issue-body.md`、`support-report.json`にはstatus、diagnosis、証跡の
hashが含まれますが、map geometry、bag、raw log、parameter内容、正確なlocal path、
credentialのようなcommand値は含まれません。3ファイルをすべて読んでから、必要な
部分だけをGitHub issueへ添付します。生成や添付は自動では行われません。

### 日本語のsupport reportとvalidation reportを分ける

困っているrunの原因を伝える`support report`と、利用者が公開手順を自分で実行した結果を
記録する`validation report`は別のものです。目的に合わせて次のどちらか1つを選び、supportの
診断をvalidationの証拠として流用しません。

| 目的 | 安全な選択 | 公開するもの | それが意味しないこと |
| --- | --- | --- | --- |
| 動かない、止まった、または原因を切り分けたい | `lidarslam-map support /path/to/session_bundle --json`でサニタイズ済みsupport reportを確認する | 内容を確認してprivate pathを除いたsupport reportの必要部分だけ | statusやdiagnosisが`PASS`に見えても、accepted validation evidenceではない |
| 独立validatorとして自分のrunを報告したい | 自分で公開手順を完了してから`lidarslam-map report /path/to/session_bundle`を実行する | canonical independent-validation issue formのpublic fieldsと、内容をreviewしたreceiptだけ | `READY FOR REVIEW`やissue提出だけでaccepted validationになるわけではない |

通常の`support --json`は診断・再現・支援のためのlocal-onlyな入口で、JSONを表示するだけで
ZIPを作りません。ZIPを作った場合は、生成された`README.txt`、`issue-body.md`、
`support-report.json`をすべて読み、必要な診断情報だけを共有します。local path、recovery JSON
（`map_session_recovery.json`）、map、bag、raw log、preview、trajectory、parameter、screenshot、
session bundle全体は貼りません。support reportはmaintainerが原因を調べるための資料であり、
receiptの代わりにも、公開validation reportの代わりにもなりません。

`report`はsupport ZIPを作る操作ではなく、同じsessionに保存されたPASS receiptを
再検証して独立validationへ進むためのread-only handoffです。`report --json`のhandoff JSON、
`receipt_path`、local receipt pathはlocal-onlyで、public attachmentではありません。表示された
canonical independent-validation issue formには、自分で実行したrunのrelease/commitまたは
immutable image digest、OS・architecture・ROS環境、private pathをredactしたexact command、
findingsを記入します。receiptを添付する場合は、formで名前が示された
`first_map_validation_receipt.json`を内容確認したもの1つだけにします。公開reportを提出しても、
maintainer reviewとvalidation ledgerのaccepted記録が済むまではaccepted evidenceではありません。

どちらのreportでも、identityはviewerの表示や移動する`develop` tagから推測しません。Dockerは
公開済み安定版`v0.9.0-humble`または`v0.9.0-jazzy`、sourceは公開releaseではない`v0.9.1`候補と
そのexact commit/revisionを`lidarslam-map --version`とreceiptから記録します。Dockerのsupport
report、source候補のvalidation report、別runのreceiptやhashを1つの報告へ混ぜず、足りない値を
推測せずに共有を止めます。

検証済みfirst mapを独立validatorへ渡す場合だけ、次のread-only handoffを使います。
`report`はreceipt-boundのPASSを再検証し、ZIPを作らず、GitHubへ連絡しません。

```bash
lidarslam-map report /path/to/session_bundle
```

出力されるverification summaryとsafe environment hintsを確認し、公開添付には
review済みのfirst-map receiptだけを使います。`report --json`のhandoff JSONは
local receipt pathを含むため、公開添付には使いません。

独立validator向けのissue formへ送る場合は、同じ出力に含まれるcanonical
independent-validation issue formを確認します。issue formには自分で実行したrunの
release/commitまたはimmutable image digest、実行command、OS・architecture・ROS環境、
statusを記入し、commandからprivate pathをredactします。receiptを添付する前に内容を
読み、PASSなら名前が示されたfirst-map receiptだけを添付します。map、bag、preview、
raw log、trajectory、parameter、スクリーンショットは添付しません。

`report --json`のhandoff JSONとlocal receipt pathはpublic attachmentではありません。
このcommandはuploadしないため、自分でissue formへ入力して共有します。独立validationは
公開手順を自分で実行した結果だけを対象にし、maintainerのlive step-by-step guidanceは
validationとして扱いません。詳しい判定条件は[Independent First-map Validation](external-first-map-validation.md)
を参照してください。

### 公開共有前の5項目チェック

`READY FOR REVIEW`が表示されても、session bundle全体を公開するわけではありません。同じ
sessionについて次の5項目を確認し、1つでも不明なら共有を止めます。5つすべてを確認できた
場合に限り、最後のreceiptだけを公開添付の候補にします。

```text
- [ ] 1. version/revision: `lidarslam-map --version`を実行し、receiptの
      `run.product_version`と`run.git_commit`（値がある場合）を記録した。
- [ ] 2. same session/output: `run.run_id`、`map_output`、receipt path、
      `verification.manifest_sha256`が同じsessionの証跡に結び付いている。
- [ ] 3. revalidation: `report`が`READY FOR REVIEW`を返し、
      receiptの`status: PASS`と全checkのPASSを内容確認した。
- [ ] 4. privacy: receiptの`shareability`を読み、issue formへ貼るcommandから
      private pathをredactした。handoff JSONとlocal receipt pathはlocal-onlyのままにした。
- [ ] 5. attachment: 公開添付は内容確認済みの
      `first_map_validation_receipt.json`だけにした。
```

1では移動する`develop` tagやviewerの表示からversionを推測せず、公開release、commit、または
immutable image digestを使います。2では別runのreceiptをコピーせず、`session.json`の
`map_output`とreceiptの`run.run_id`、manifest hashを同じsessionで照合します。3がrejectしたら、
receiptを編集せず、前述の「receipt再検証に失敗したときの復旧」へ戻ります。

4のhandoff JSON、receipt path、session JSON、support reportのlocal pathは、共有用の資料では
ありません。5以外に、map、bag、raw log、preview HTML、trajectory、parameter、screenshot、
session bundle全体も添付しません。issue formのversion、environment、redacted command、
verification summaryは自分のrunから転記し、receiptのJSONだけをレビュー済みの添付として使います。

### 公開共有用のreceiptテンプレート

5項目を確認した後は、`report`が表示するcanonical issue formを開き、次の
templateを自分のrunの値で埋めます。`READY FOR REVIEW`とreceiptの`PASS`が確認できない場合は、
このtemplateを公開用に使いません。

```text
公開ドキュメント経路: <Docker First Map / Source quickstart / Own-bag golden path>
release/commit/image digest: <immutable release, commit, or image digest>
environment: <OS> / <architecture> / <ROS> / <install method>
exact command (private paths redacted):
<public documentation command with private paths removed>
result: PASS — verified first map completed
verification summary:
manifest_status=succeeded
diagnosis_status=success
autoware_status=PASS
manifest_sha256=<64 lowercase hex characters>
findings:
<what was unclear, slow, surprising, broken, or helpful>
attachment: first_map_validation_receipt.json (reviewed)
```

### 日本語の公開reportのfield provenanceを確認する

templateの各fieldは、report作成者が自分のrunから入力する値と、同じsessionのreceiptまたは
`report` handoffから転記する値に分かれます。値の出どころが確認できないfieldは推測せず、
reportを提出しません。

| fieldの種類 | 自分で確認する出どころ | 該当するfield | 入力ルール |
| --- | --- | --- | --- |
| operator-supplied public fields | 自分が実行した公開手順、環境、観察 | 公開ドキュメント経路、`environment`、private pathをredactしたexact command、`findings` | 自分のrunの事実だけを書く。viewerの表示やexampleの値で補わず、local path・credential・raw artifactを除く |
| receipt-derived validation fields | 同じrunの`report` handoffと、名前が示されたreview済みreceipt | `release/commit/image digest`、`result`、verification summary、`manifest_sha256`、receipt attachment | receipt/handoffの値をそのまま照合して転記する。別session、別output、別versionの値を混ぜない |
| review / acceptance status | 公開issueの状態、maintainer review、validation ledger | `READY FOR REVIEW`、public report submitted、maintainer review、accepted ledger evidence | 自分でacceptedと名付けない。ledgerの明示的なaccepted記録がない間はaccepted validationと書かない |

次の順序で同じsessionを照合します。まず`lidarslam-map --version`とreceiptの
`run.product_version`・`run.git_commit`（値がある場合）を比較し、次に`run.run_id`、
`map_output`、`verification.manifest_sha256`をhandoff・session・receiptで突き合わせます。
`result: PASS`、verification summary、hash、またはreceipt pathがmissing、`UNAVAILABLE`、
mismatch、example-only、viewer-onlyの場合は、reportを公開せず`Details:`、`Next:`、または
保存されたretry指示へ戻ります。receipt、manifest、session JSONを編集して値を合わせません。

identityが曖昧な場合も停止します。Dockerなら公開済み`v0.9.0-humble`または`v0.9.0-jazzy`、
sourceなら未公開`v0.9.1`候補とexact commit/revisionを、receiptまたはhandoffで確認できた場合
だけ記録します。`develop` tag、viewer、架空例からidentityやhashを作りません。handoff JSON、
local path、recovery JSON、map、bag、raw log、preview、trajectory、parameter、screenshot、
session bundle全体はreportにも添付にも含めず、公開添付はreview済みreceiptだけにします。

`release/commit/image digest`はreceiptの`run.product_version`と`run.git_commit`またはhandoffの
release referenceに対応させます。`environment`、redacted command、findingsは自分で入力する
公開fieldです。`verification summary`はreceipt markdownのPASS blockから転記し、hashを推測・
編集しません。

`report --json`のhandoff、`receipt_path`、`markdown_path`、`session.json`、support reportの
local pathはtemplateに貼りません。map、bag、raw log、preview、trajectory、parameter、
screenshotも添付せず、PASSを確認して内容をreviewした
`first_map_validation_receipt.json`だけをpublic attachmentにします。

### 日本語のvalidation reportのfindingsを安全に書く

`findings`は、自分のrunで気付いた一件の観察をmaintainerが再現・改善するための
operator-supplied fieldです。`result`、verification summary、manifest hash、receipt attachment、
review statusを変更したり、root causeやaccepted statusを推測したりするfieldではありません。
一つの観察につき、次の4項目を自分の値で埋めます。

```text
findings:
step: <公開ドキュメントのstepまたはcommand名。private pathは書かない>
expected: <公開ドキュメントが示す期待動作を1つ>
observed: <自分のrunで実際に観察した動作を1つ>
impact: <first-run userが何に困るか、または再現に何が必要か>
```

`step`には`Docker First Map`、`source quickstart`、`Session summary`など公開名を使い、
必要なら安定した`reason.code`を添えます。local path、credential、bag/map名、raw logの行、
preview HTML、screenshot、session bundleの内容を貼らず、viewerで見えた症状だけから診断を
断定しません。`expected`は公開手順から、`observed`は自分のrunから、`impact`は利用者への
影響または再現条件から書き分けます。root causeが不明なら`不明`と書き、support reportへ
戻します。

複数の観察がある場合も、4項目のblockを観察ごとに分けます。同じreceiptのhashや`PASS`を
findingsへ再掲して証拠を増やしたり、findingを理由に`result`、verification summary、
`manifest_sha256`、receipt JSON、review statusを書き換えたりしません。自分で公開手順を
完了していない、またはmaintainerのlive guidanceだけに基づく観察は独立validation reportへ
入れず、前述のサニタイズ済みsupport reportとして扱います。

### 日本語のvalidation reportのfinding follow-upを安全に行う

`findings`が`unresolved`または`rejected`になった後は、元のreportとreceiptを編集せず、
まず保存された`Details:`、`Next:`、`retry.command`を読みます。次の二つの経路を混ぜずに
選びます。

| 経路 | 使う条件 | 進め方 | してはいけないこと |
| --- | --- | --- | --- |
| support follow-up | 元のrunの原因を切り分ける、またはretryの結果を相談する | 保存された`Details:`/`Next:`を確認し、同じpinned setupの`retry.command`が使える場合だけlocalで編集せず実行する。新しい`retry.output_dir`のfresh outputをサニタイズ済みsupport reportへ要約する | 元のreport、receipt、manifest hash、review statusを編集する。support reportをaccepted validationにする |
| new independent validation | 自分で公開手順を最初から完了し、独立した新しいrunの結果を報告する | 公開routeを新しいoutputとsessionで実行し、そのrunに結び付いたreportとreview済みreceiptを1組だけ準備する | 古いmap、session、receipt、hashを新runへコピーする。maintainerのlive guidanceだけの結果を独立validationと呼ぶ |

support follow-upで公開するのは、private pathを除いた観察と再現条件だけです。保存された
`retry.command`にlocal pathやcredentialが含まれる場合は、command全体をreportへ貼らず、安定した
`reason.code`、command名、サニタイズ済みの`Details:`/`Next:`だけを記録します。retryは新しい
outputへ書き、元のoutput、report、receipt、`manifest_sha256`を上書きしません。次のような
形で一件のfollow-upを要約できます。

```text
follow-up route: support follow-up
original report/receipt: unchanged
reason.code: <保存された安定code>
Details: <private pathを除いた保存済みの説明>
Next: <private pathを除いた保存済みの次の操作>
fresh output: <新しいoutputで観察した事実と影響>
review status: unresolved / retrying — not accepted validation
```

new independent validationへ進む場合も、元のreportとreceiptはそのまま保持します。Dockerは
公開済み`v0.9.0-humble`または`v0.9.0-jazzy`、sourceは未公開`v0.9.1`候補とexact
commit/revisionを、その新しいsessionとreceiptから確認します。identity、session、または
同じrunへの結び付きを確認できない場合は停止し、viewerの表示、古いreceipt、架空のhashで
補いません。新しいreportを提出しても、maintainer reviewとledgerのaccepted記録までは
`READY FOR REVIEW`または`unresolved`のままで、accepted validationとは書きません。

一つのrunにつきreportとreceiptは1組だけにし、同じfollow-upを複数issueへ重複添付しません。
map、bag、raw log、preview、trajectory、parameter、screenshot、session bundle全体、handoff
JSON、local receipt pathは公開せず、前述のサニタイズ済みsupport reportまたは内容をreviewした
receiptだけを使います。

### 日本語のvalidation reportのfollow-up証跡を一組で保つ

follow-upを監査するときは、元のrunの`report + reviewed receipt`、follow-upの要約、別runの
independent-validation reportを別のものとして記録します。follow-upの要約は新しいreceiptや
新しいaccepted evidenceではありません。元のreport、receipt、`manifest_sha256`、review status
を変えず、同じrunなら元の一組を参照し、新しいrunならそのrunだけの一組を作ります。

| 記録 | 意味 | receiptの扱い | 監査で確認すること |
| --- | --- | --- | --- |
| original report/receipt pair | 一つのrunから作った公開reportと内容をreviewしたreceipt | そのまま保持し、再生成・編集・hash変更をしない | Docker/source identity、session/output、reportとreceiptの対応が同じrunである |
| follow-up note | 元のfinding後に行ったsupport相談またはretryの要約 | 新しいreceiptを作らず、元の一組に対するnoteとして扱う | route、保存済み`reason.code`、sanitized `Details:`/`Next:`、fresh-output facts、review statusが分かる |
| new independent-validation report | 公開手順を自分で完了した別runの報告 | 新しいrunに結び付くreportとreview済みreceiptを1組だけ使う | 古い一組を新runへ混ぜず、acceptedはledgerの明示記録がある場合だけ書く |

次のblockはfollow-up noteの入力用です。local path、credential、raw command、receipt hashを
そのまま貼らず、保存された情報を公開用にサニタイズしてから使います。

```text
follow-up audit:
original report/receipt pair: <one unchanged pair; no local path>
original identity: <v0.9.0 Docker identity or v0.9.1 source candidate + exact revision>
follow-up route: <support follow-up / new independent validation>
reason.code: <saved stable code>
Details: <sanitized saved explanation>
Next: <sanitized saved next action>
fresh-output facts: <one fact and its user impact; no raw artifact>
review status: <unresolved / retrying / READY FOR REVIEW — not accepted>
duplicate check: <no duplicate issue or session artifact>
```

同じrunのsupport follow-upでは、`original report/receipt pair`を1組のまま保ち、`fresh-output
facts`だけを新しいoutputから追記します。`Details:`、`Next:`、`retry.command`の値が元の
sessionやlocal pathを含む場合は、公開にはstable code、command名、影響だけを残します。
別runのindependent validationでは、Dockerなら公開済み`v0.9.0-humble`または`v0.9.0-jazzy`、
sourceなら未公開`v0.9.1`候補とexact commit/revisionを新しいsessionとreceiptから確認し、
古いreport、receipt、map、hashをコピーしません。

identity、session、output、またはreportとreceiptが同じrunに結び付くことを確認できない場合は
監査を止めます。viewerの見た目、古いreceipt、架空のhashで不足を埋めず、acceptedや
independent validationとも書きません。support相談やmaintainerのlive guidanceだけを行った
場合は、follow-up noteまたはサニタイズ済みsupport reportとして扱い、公開validation evidence
の一組に昇格させません。

### 日本語のvalidation reportのfollow-up要約を監査可能にする

maintainerがfollow-up要約を受け取ったら、要約だけを見てPASSやacceptedを決めず、元の
`report + reviewed receipt`と照合します。照合は次の順序で行い、どれか一つでも確認できない
場合は`STOP`にします。

| 順序 | 確認するもの | 一致・不足の判定 |
| --- | --- | --- |
| 1. identity | Dockerの公開済み`v0.9.0-humble`/`v0.9.0-jazzy`、またはsourceの未公開`v0.9.1`候補とexact commit/revision | viewer、`develop` tag、example-only値から補わない。missing/mismatchならSTOP |
| 2. same run | 元のreport、receipt、session/output、`manifest_sha256`が同じrunに結び付く | local pathやsession bundleを公開せず、同じrunを確認できなければSTOP |
| 3. route | `support follow-up`、または新しい`independent validation`のどちらか | noteを新しいreport/receiptに変換せず、別runなら古い一組と混ぜない |
| 4. provenance | `reason.code`、sanitized `Details:`/`Next:`、fresh-output factsの出どころ | receipt-derived result、verification、hash、review statusをfollow-up factsで上書きしない |
| 5. review status | `unresolved`、`retrying`、`READY FOR REVIEW`、maintainer review、ledger | ledgerの明示的なaccepted記録がない限りacceptedと分類しない |

照合結果は、次の3分類だけを使います。`MATCHED FOLLOW-UP`は、元の一組が不変で、同じrunに
結び付いたサニタイズ済みnoteとして扱える場合だけです。`NEW RUN`は、公開手順を自分で完了した
別sessionのreportとreceiptが新しく1組そろい、古い一組と混ざっていない場合だけです。identity、
session、output、route、またはfieldの出どころが不明なら`STOP`であり、viewerの見た目やnoteの
文面から推測して先に進みません。

```text
follow-up audit disposition:
original pair: matched / missing / mismatch
identity: <public Docker identity or source candidate + exact revision>
same run/output: matched / stop
route: support follow-up / new independent validation
provenance: <reason.code + sanitized Details:/Next: + fresh-output facts>
evidence change: none — original report, receipt, and hash unchanged
review status: <unresolved / retrying / READY FOR REVIEW — not accepted>
disposition: MATCHED FOLLOW-UP / NEW RUN / STOP
```

このdispositionはmaintainerの監査メモであり、新しいvalidation resultやreceiptではありません。
`MATCHED FOLLOW-UP`でもoriginal report/receipt pairは1組のまま、`NEW RUN`でも新しいrunの
report/receiptは1組だけにします。複数issueへ同じnoteやsession artifactを重複添付せず、map、
bag、raw log、preview、trajectory、parameter、screenshot、handoff JSON、local receipt path、
session bundle全体は公開しません。`STOP`の結果をaccepted、再現成功、またはroadmap evidenceと
書かず、保存された`Details:`、`Next:`、`retry.command`へ戻ります。

### 日本語のvalidation reportのfollow-up監査結果を安全に分類する

分類の後に何をしてよいかも固定します。`MATCHED FOLLOW-UP`、`NEW RUN`、`STOP`は操作の許可証
ではなく、元のevidenceをどう扱うかを示す監査結果です。どの分類でも、receipt、
`run_manifest.json`、`session.json`、`manifest_sha256`を手編集したり、follow-up noteを
accepted validationへ変換したりしません。

| 分類 | 許可される対応 | evidenceの扱い | 公開status |
| --- | --- | --- | --- |
| `MATCHED FOLLOW-UP` | 元のrunのsupport相談、または保存された`Details:`/`Next:`/`retry.command`に従うsafe retry。fresh outputの事実はサニタイズ済みnoteへ追記する | original report/receipt pairは変更せず、新しいreceiptを作らない | `unresolved`、`retrying`、または`READY FOR REVIEW` — not accepted |
| `NEW RUN` | exact identity、session、outputが確認できる新しいindependent validationだけを別runとして確認する | 新しいrunのreport + reviewed receiptを1組だけ作り、古いpairと混ぜない | maintainer review前は`READY FOR REVIEW` — acceptedはledgerの明示記録後だけ |
| `STOP` | サニタイズ済みsupportを依頼し、保存された`Details:`、`Next:`、`retry.command`へ戻る | original evidenceはそのまま保持し、新しいreportやreceiptを作らない | `STOP` — not accepted、公開validation noteを作らない |

`MATCHED FOLLOW-UP`では、同じrunの原因確認や保存済みretryだけを行います。retryで新しいoutputが
得られても、元のreceiptのresult、verification、hash、review statusを上書きせず、観察した事実と
影響だけをサニタイズしてnoteにします。`NEW RUN`では、Dockerなら公開済み`v0.9.0-humble`または
`v0.9.0-jazzy`、sourceなら未公開`v0.9.1`候補とexact commit/revisionを確認できる場合だけ、
新しいreport + reviewed receiptを一組にします。identity、session、outputのどれかが不足する
場合は`STOP`です。

`STOP`では、文面やviewerの見た目から原因、PASS、再現成功を推測せず、public validation noteを
作りません。supportへ渡すときもprivate path、credential、bag、map、raw log、session bundleを
含めず、保存された`Details:`/`Next:`と安全なretry条件だけを使います。acceptedは、maintainer
reviewとledgerの明示的なaccepted記録がある場合だけ書きます。どの分類でも一つのrunにつき
report + reviewed receiptは一組だけで、複数issueへ同じnoteやsession artifactを重複添付しません。

次のblockは、分類後の許可された対応を記録するためのものです。local pathやprivate artifactは
書きません。

```text
follow-up action:
disposition: MATCHED FOLLOW-UP / NEW RUN / STOP
allowed action: <saved Details:/Next:/retry.command or new-run review>
original evidence: unchanged
new evidence: none / one new run report + reviewed receipt / none
next step: <sanitized Details: or Next:; no private path>
public status: unresolved / READY FOR REVIEW — not accepted / STOP — not accepted
duplicate check: <one pair per run; no duplicate issue or session artifact>
```

このaction blockは新しいvalidation resultや受理記録ではありません。`MATCHED FOLLOW-UP`は元の
一組に対するnote、`NEW RUN`は新しい一組、`STOP`は保留とsupport依頼です。分類後の操作が元の
identity、session、outputに結び付かない場合や、既存のpairを編集・複製しそうな場合は、操作を
進めず`STOP`に戻します。

### 日本語のvalidation reportのfollow-up分類後の対応を監査可能にする

分類を記録しただけでは、実際に何をしたかは決まりません。`action result`を別に記録し、
許可された操作だけが行われたことを確認します。action resultが分類と一致しない場合や、
identity、session、outputを再確認できない場合は、元の証跡を編集せず`STOPPED`として扱います。

| disposition | action result | 許可されるevidence変更 | 公開status |
| --- | --- | --- | --- |
| `MATCHED FOLLOW-UP` | `NOTE ONLY`、`SUPPORT REQUESTED`、または`RETRY STARTED` | 変更なし。元のpairを参照し、retryのfresh factsだけをnoteへ追記する | `unresolved`、`retrying`、または`READY FOR REVIEW` — not accepted |
| `NEW RUN` | `NEW PAIR PREPARED` | exact identity、session、outputがそろった新runのreport + reviewed receiptを1組だけ作る | `READY FOR REVIEW` — acceptedはledgerの明示記録後だけ |
| `STOP` | `STOPPED` | 変更なし。新しいreport、receipt、公開validation noteを作らない | `STOP` — not accepted |

`MATCHED FOLLOW-UP`で`NOTE ONLY`、`SUPPORT REQUESTED`、または`RETRY STARTED`を記録した場合も、
元のreport、receipt、hash、review statusは変更しません。`NEW RUN`の`NEW PAIR PREPARED`は、
Dockerなら公開済み`v0.9.0-humble`または`v0.9.0-jazzy`、sourceなら未公開`v0.9.1`候補と
exact commit/revisionを新しいsessionから確認できる場合だけ許可します。古いpairのコピー、
receiptの再利用、同じrunの二重pairは認めません。

`STOP`の`STOPPED`は、support依頼または保存された`Details:`/`Next:`/`retry.command`へ戻る
ための結果です。原因、PASS、再現成功、acceptedを推測せず、private path、credential、bag、
map、raw log、session bundleを公開しません。acceptedは、maintainer reviewとledgerの明示的な
accepted記録がある場合だけ書きます。

次のblockは、分類後の実行結果を記録するためのものです。local pathやprivate artifactは書かず、
`new artifact count`は許可された一組の数だけを記録します。

```text
follow-up action audit:
disposition: MATCHED FOLLOW-UP / NEW RUN / STOP
action result: NOTE ONLY / SUPPORT REQUESTED / RETRY STARTED /
               NEW PAIR PREPARED / STOPPED
identity/session/output: matched / missing / mismatch
allowed evidence change: none / one new report + reviewed receipt / none
original evidence: unchanged
new artifact count: 0 / one report + reviewed receipt / 0
public status: unresolved / READY FOR REVIEW — not accepted / STOP — not accepted
reason.code: <saved stable code>
Details: <sanitized explanation; no private path>
Next: <sanitized next action; no private path>
handoff: <support / maintainer review / stop>
audit result: completed / STOPPED — no acceptance
```

実行結果がこのblockの許可範囲と違った場合は、既存のreportやreceiptを書き換えて整合させず、
新しい監査メモを`STOPPED`として残します。一つのrunにつきreport + reviewed receiptは一組だけ、
複数issueへ同じnoteやsession artifactは重複添付しません。`NEW RUN`でもledgerの受理前は
`READY FOR REVIEW`であり、`MATCHED FOLLOW-UP`や`STOP`をindependent validationへ昇格させません。

### 日本語のvalidation reportのfollow-up実行結果を安全に引き継ぐ

handoffは、次の担当者へ監査結果を渡すためのroutingであり、acceptanceやreceipt編集の許可では
ありません。action resultごとに引き継ぎ先と公開statusを固定すると、外部contributorはprivate
artifactやmaintainerのlive guidanceに頼らず、同じ安全境界で相談・reviewを続けられます。

| action result | handoff target | handoff status | 引き継ぎ先がしてよいこと | してはいけないこと |
| --- | --- | --- | --- | --- |
| `NOTE ONLY` / `SUPPORT REQUESTED` / `RETRY STARTED` | support | `READY FOR SUPPORT` — not accepted | サニタイズ済み`reason.code`、`Details:`、`Next:`、fresh factsを確認し、保存済みretry条件を案内する | 新しいaccepted evidenceやreceiptを作る。live guidanceだけをindependent validationと呼ぶ |
| `NEW PAIR PREPARED` | maintainer review | `READY FOR REVIEW` — not accepted | 新runのidentity、session、output、report + reviewed receiptを照合し、不足なら差し戻す | 古いpairをコピーする。ledger前にaccepted、PASS、roadmap evidenceと書く |
| `STOPPED` | stop / recovery | `STOP` — not accepted | 不足・mismatchをサニタイズして返し、保存済み`Details:`/`Next:`/`retry.command`へ戻す | 原因や再現成功を推測する。公開validation noteや新しいreceiptを作る |

`READY FOR SUPPORT`はsupportの入口であり、結果の受理ではありません。`READY FOR REVIEW`でも、
maintainer reviewとledgerの明示的なaccepted記録までは受理されません。`STOP`は失敗を隠すstatus
ではなく、identity、session、output、またはfieldの出どころが確認できないときの再確認経路です。
どのhandoffでも元のreport、receipt、hash、review statusは変更せず、Dockerの公開済み
`v0.9.0-humble`/`v0.9.0-jazzy`とsourceの未公開`v0.9.1`候補のidentity境界を保ちます。

次のblockは、担当者へ渡す最小情報を記録するためのものです。roleだけを使い、local path、
credential、bag、map、raw log、session bundleは書きません。

```text
follow-up handoff:
original audit: <sanitized stable code; no private path>
disposition: MATCHED FOLLOW-UP / NEW RUN / STOP
action result: NOTE ONLY / SUPPORT REQUESTED / RETRY STARTED /
               NEW PAIR PREPARED / STOPPED
handoff target: support / maintainer review / stop
handoff status: READY FOR SUPPORT / READY FOR REVIEW — not accepted /
                STOP — not accepted
original evidence: unchanged
new evidence: none / one new report + reviewed receipt / none
owner role: operator / support / maintainer
next review: <date or not scheduled>
reason.code: <saved stable code>
Details: <sanitized explanation>
Next: <sanitized next action>
privacy check: <no private path or artifact>
duplicate check: <one handoff per action audit; no duplicate issue or session artifact>
```

引き継ぎ先がこのblockのstatusや許可範囲を変更したい場合は、既存のaudit blockを上書きせず、
新しいaction resultを別の監査メモとして記録します。一つのaction auditにつきhandoffは一つ、
一つのrunにつきreport + reviewed receiptは一組だけです。missing/mismatchを埋めるための架空の
hash、viewerの見た目、古いreceiptは使わず、`STOP`から安全なsupport/retryへ戻ります。

### validation reportのreview statusを区別する

`READY FOR REVIEW`は、元のsessionに対するlocal-onlyの再検証が完了し、canonical issue formへ
進める状態です。これは公開報告の提出済みでも、独立validationのacceptedでもありません。
次の状態を混同しないでください。

| 状態 | 意味 | 次の操作 |
| --- | --- | --- |
| local `READY FOR REVIEW` | 同じsessionのreceipt、manifest、diagnosis、verificationを再確認でき、公開レビューへ進めるlocal handoffがある | 5項目チェックとreceiptの内容確認を行い、private pathをredactして1つのreportを準備する |
| public report submitted | canonical issue formにreportとreview済みreceiptを提出した | 提出済みのreportとreceiptを保持し、maintainer reviewを待つ。提出だけではacceptedにならない |
| maintainer review | 公開されたidentity、report、receipt、privacy、ledger条件をmaintainerが確認している | 公開手順と保存された`Next:`に従い、liveなstep-by-step validation helpを依頼せず、既存receiptを編集しない |
| accepted ledger evidence | maintainer reviewと公開ledgerの受理条件が通り、ledgerにacceptedとして記録された | この明示的なledger記録がある場合だけ、独立validationのaccepted evidenceとして扱う |
| unresolved / rejected | 情報不足、receipt mismatch、未解決、または明示的なrejectで、ledgerの受理条件を満たしていない | acceptedと表示せず、元の証跡を保持して`Details:`、`Next:`、保存されたretry指示を確認する |

localの`READY FOR REVIEW` handoffやpublic receiptは、公開reviewとledgerの受理条件が通るまで
accepted validationではありません。公開issueを提出しただけ、maintainerが確認中であるだけ、
receiptの`status: PASS`だけ、またはviewerにmapが表示されたことだけから、acceptedやroadmapの
matrix evidenceを推測しません。

提出後は、同じ内容のreportと同じreview済みreceiptを1組だけ使います。receipt、
`run_manifest.json`、`session.json`、hashを手編集して受理条件に合わせたり、同じevidenceを
複数issueへ重複添付したりしません。`READY FOR REVIEW`のhandoff JSON、local receipt path、
session bundle、map、bag、raw log、previewは引き続きlocal-onlyです。reviewがunresolvedまたは
rejectedになった場合もmapが削除されたという意味ではなく、保存された安全なretryや新しい
outputの指示がある場合だけそれに従います。

### 日本語のprivacy-safe validation report例

次は、公開formの項目と値の出どころを理解するための説明用の架空例です。実際のrun、receipt、
ledgerから作ったものではないため、実際のvalidation result、review済みevidence、accepted
ledger evidenceではありません。値やhashを自分のreportへコピーせず、自分の
`report`とreview済みreceiptから実際の値を確認します。

```text
[説明用の架空例 — not a real validation result / not accepted ledger evidence]
公開ドキュメント経路: Docker First Map (example)
release/commit/image digest: example-image@sha256:<example-only-64-hex-digest>
environment: Ubuntu 22.04 / amd64 / ROS 2 Humble / Docker
exact command (private paths redacted):
docker run --rm ghcr.io/rsasaki0109/lidar_slam_ros2:<immutable-example-release>
result: PASS — verified first map completed (example only)
verification summary:
manifest_status=succeeded
diagnosis_status=success
autoware_status=PASS
manifest_sha256=<example-only; copyしない>
findings:
説明用: 固定された公開経路の前提を確認できた
attachment: first_map_validation_receipt.json (reviewed) [example only]
review status: EXAMPLE ONLY — not submitted / not maintainer-reviewed / not accepted
```

この例でreport作成者が入力する`operator-supplied public fields`は、公開ドキュメント経路、
environment、private pathをredactしたcommand、findingsです。`result`、verification summary、
`manifest_sha256`、receipt attachmentは実際のreview済みreceiptから転記する
`receipt-derived fields`であり、見た目やviewerから補いません。架空の`example-only`値を
実際のevidenceとして扱わず、実runのhashやrelease identityが確認できない場合はreportを
提出しません。

実際のreportにもlocal path、map、bag、raw log、preview、session bundleは含めません。公開添付は
前述の条件を満たして内容をreviewしたreceiptだけです。`review status`が
`not submitted`、`not maintainer-reviewed`、または`not accepted`の例は、GitHub issueや
validation ledgerの証拠として使いません。

## 詳細

このページは最短経路だけを示します。すべてのoption、対応input、校正、復旧、
自動化contractは英語版の[Getting Started](getting-started.md)と
[Operator Workflows](workflows.md)を参照してください。正規コマンドや安全境界に
差がある場合は英語版が優先されます。
