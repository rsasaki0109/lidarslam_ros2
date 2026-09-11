# Bash completion for the installed lidarslam-map product command.

_LIDARSLAM_MAP_COMMANDS='demo start sessions compare report support doctor setup run inspect view edit merge migrate-manifest rollback-plan'
_LIDARSLAM_MAP_GLOBAL_OPTIONS='--help --help-all --version'
_LIDARSLAM_MAP_DEMO_OPTIONS='--help --help-all --data-dir --output-dir --output --viewer --min-free-space-gib --dry-run --resume --json'
_LIDARSLAM_MAP_START_OPTIONS='--help --help-all --profile --output-dir --map-output-dir --accept-profile-extrinsics --lidar-to-base --imu-to-base --base-frame --lidar-frame --imu-frame --json --yes --dry-run --editable --viewer --min-free-space-gib --verification'
_LIDARSLAM_MAP_SESSIONS_OPTIONS='--help --help-all --status --limit --viewer --json'
_LIDARSLAM_MAP_COMPARE_OPTIONS='--help --help-all --output --viewer --json'
_LIDARSLAM_MAP_REPORT_OPTIONS='--help --help-all --json'
_LIDARSLAM_MAP_SUPPORT_OPTIONS='--help --help-all --output --json --first-map'
_LIDARSLAM_MAP_DOCTOR_OPTIONS='--help --help-all --json --public-json --demo-dir --min-free-space-gib'
_LIDARSLAM_MAP_SETUP_OPTIONS='--help --help-all --profile --output-dir --map-output-dir --accept-profile-extrinsics --lidar-to-base --imu-to-base --base-frame --lidar-frame --imu-frame --json'
_LIDARSLAM_MAP_RUN_OPTIONS='--help --help-all --profile --output-dir --lidarslam-param --rko-param --base-frame --lidar-frame --imu-frame --min-free-space-gib --dry-run --editable --resume --guided --yes --viewer --autoware-core-dir --work-dir --viewer-run-dir --viewer-rebuild --auto-exit-secs --verification --no-verify-map'
_LIDARSLAM_MAP_INSPECT_OPTIONS='--help --help-all --bag --symptom --json --write'
_LIDARSLAM_MAP_VIEW_OPTIONS='--help --help-all --viewer --no-open --preview-dir --autoware-core-dir --work-dir --runtime-dir --rebuild --auto-exit-secs'
_LIDARSLAM_MAP_EDIT_OPTIONS='--help --help-all --plan --output-dir --dry-run --backend-input --params --setup --json'
_LIDARSLAM_MAP_MERGE_OPTIONS='--help --help-all --output-dir --merge-voxel-size --alignment-voxel-size --max-alignment-points --max-median-error --max-p90-error --min-overlap --initial-transform --dry-run --json'
_LIDARSLAM_MAP_MIGRATE_MANIFEST_OPTIONS='--help --help-all --output --verification --json'
_LIDARSLAM_MAP_ROLLBACK_PLAN_OPTIONS='--help --help-all --json'

_lidarslam_map_complete_directories() {
  local current="$1"
  if declare -F _filedir >/dev/null 2>&1; then
    _filedir -d
  else
    COMPREPLY=($(compgen -d -- "$current"))
  fi
}

_lidarslam_map_complete_files() {
  local current="$1"
  if declare -F _filedir >/dev/null 2>&1; then
    _filedir
  else
    COMPREPLY=($(compgen -f -- "$current"))
  fi
}

_lidarslam_map_complete() {
  local current previous command options
  COMPREPLY=()
  current="${COMP_WORDS[COMP_CWORD]}"
  previous="${COMP_WORDS[COMP_CWORD - 1]:-}"
  command="${COMP_WORDS[1]:-}"

  if (( COMP_CWORD == 1 )); then
    COMPREPLY=($(compgen -W \
      "${_LIDARSLAM_MAP_COMMANDS} ${_LIDARSLAM_MAP_GLOBAL_OPTIONS}" \
      -- "$current"))
    return
  fi

  case "$previous" in
    --profile)
      COMPREPLY=($(compgen -W \
        'rko_lio_graph_public_path rko_lio_graph_mid360_preset pointcloud_gnss_smoke packet_applanix_smoke' \
        -- "$current"))
      return
      ;;
    --verification)
      COMPREPLY=($(compgen -W 'required off' -- "$current"))
      return
      ;;
    --status)
      COMPREPLY=($(compgen -W \
        'all running verified unverified action_required' \
        -- "$current"))
      return
      ;;
    --symptom)
      COMPREPLY=($(compgen -W \
        'map-spins-or-spirals pose-drifts-or-oscillates map-stops-early map-is-too-sparse map-is-not-visible' \
        -- "$current"))
      return
      ;;
    --viewer)
      if [[ "$command" == 'view' ]]; then
        COMPREPLY=($(compgen -W 'browser autoware foxglove' -- "$current"))
      elif [[ "$command" == 'demo' || "$command" == 'sessions' || "$command" == 'compare' ]]; then
        COMPREPLY=($(compgen -W 'browser none' -- "$current"))
      else
        COMPREPLY=($(compgen -W 'none browser autoware foxglove' -- "$current"))
      fi
      return
      ;;
    --data-dir|--output-dir|--map-output-dir|--bag|--demo-dir|--preview-dir|--autoware-core-dir|--work-dir|--viewer-run-dir|--runtime-dir|--backend-input)
      _lidarslam_map_complete_directories "$current"
      return
      ;;
    --output|--lidarslam-param|--rko-param|--plan|--params|--setup)
      _lidarslam_map_complete_files "$current"
      return
      ;;
  esac

  case "$command" in
    demo)
      options="$_LIDARSLAM_MAP_DEMO_OPTIONS"
      ;;
    start)
      options="$_LIDARSLAM_MAP_START_OPTIONS"
      ;;
    sessions)
      options="$_LIDARSLAM_MAP_SESSIONS_OPTIONS"
      ;;
    compare)
      options="$_LIDARSLAM_MAP_COMPARE_OPTIONS"
      ;;
    report)
      options="$_LIDARSLAM_MAP_REPORT_OPTIONS"
      ;;
    support)
      options="$_LIDARSLAM_MAP_SUPPORT_OPTIONS"
      ;;
    doctor)
      options="$_LIDARSLAM_MAP_DOCTOR_OPTIONS"
      ;;
    setup)
      options="$_LIDARSLAM_MAP_SETUP_OPTIONS"
      ;;
    run)
      options="$_LIDARSLAM_MAP_RUN_OPTIONS"
      ;;
    inspect)
      options="$_LIDARSLAM_MAP_INSPECT_OPTIONS"
      ;;
    view)
      options="$_LIDARSLAM_MAP_VIEW_OPTIONS"
      ;;
    edit)
      options="$_LIDARSLAM_MAP_EDIT_OPTIONS"
      ;;
    merge)
      options="$_LIDARSLAM_MAP_MERGE_OPTIONS"
      ;;
    migrate-manifest)
      options="$_LIDARSLAM_MAP_MIGRATE_MANIFEST_OPTIONS"
      ;;
    rollback-plan)
      options="$_LIDARSLAM_MAP_ROLLBACK_PLAN_OPTIONS"
      ;;
    *)
      return
      ;;
  esac

  if [[ "$current" == -* ]]; then
    COMPREPLY=($(compgen -W "$options" -- "$current"))
    return
  fi
  if [[ "$command" == 'rollback-plan' ]]; then
    _lidarslam_map_complete_files "$current"
  else
    _lidarslam_map_complete_directories "$current"
  fi
}

complete -F _lidarslam_map_complete lidarslam-map
