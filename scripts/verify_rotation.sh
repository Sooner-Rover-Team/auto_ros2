#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  scripts/verify_rotation.sh [options]

Checks whether rover heading is changing in TF while you rotate in place.
It samples tf2 yaw from odom -> base_link and reports PASS/FAIL.

Options:
  --duration <seconds>       Sampling time (default: 12)
  --min-yaw-change-rad <r>   Minimum yaw span to pass (default: 0.20)
  --from-frame <frame>       Parent frame (default: odom)
  --to-frame <frame>         Child frame (default: base_link)
  --no-source                Skip sourcing install/setup.bash
  -h, --help                 Show this help
EOF
}

duration="12"
min_yaw_change_rad="0.20"
from_frame="odom"
to_frame="base_link"
auto_source=1

while [[ $# -gt 0 ]]; do
    case "$1" in
    --duration)
        duration="${2:-}"
        shift 2
        ;;
    --min-yaw-change-rad)
        min_yaw_change_rad="${2:-}"
        shift 2
        ;;
    --from-frame)
        from_frame="${2:-}"
        shift 2
        ;;
    --to-frame)
        to_frame="${2:-}"
        shift 2
        ;;
    --no-source)
        auto_source=0
        shift
        ;;
    -h | --help)
        usage
        exit 0
        ;;
    *)
        echo "Unknown option: $1" >&2
        usage >&2
        exit 2
        ;;
    esac
done

if ! [[ "${duration}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "--duration must be numeric" >&2
    exit 2
fi

if ! [[ "${min_yaw_change_rad}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "--min-yaw-change-rad must be numeric" >&2
    exit 2
fi

if [[ "${auto_source}" -eq 1 ]]; then
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    workspace_root="$(cd "${script_dir}/.." && pwd)"
    setup_file="${workspace_root}/install/setup.bash"
    if [[ -f "${setup_file}" ]]; then
        set +u
        # shellcheck disable=SC1090
        source "${setup_file}"
        set -u
    else
        echo "warning: ${setup_file} not found, assuming ROS env already sourced" >&2
    fi
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 not found in PATH. Source your ROS environment first." >&2
    exit 1
fi

if ! command -v timeout >/dev/null 2>&1; then
    echo "timeout command not found in PATH." >&2
    exit 1
fi

echo "Sampling TF yaw (${from_frame} -> ${to_frame}) for ${duration}s..."
echo "Rotate the rover in place now."

if ! result="$(
    timeout "${duration}" ros2 run tf2_ros tf2_echo "${from_frame}" "${to_frame}" 2>/dev/null \
        | awk -v min_span="${min_yaw_change_rad}" '
            function trim(s) {
                gsub(/^[ \t]+|[ \t]+$/, "", s)
                return s
            }
            function abs(v) {
                return v < 0 ? -v : v
            }
            /Rotation: in RPY/ {
                if (match($0, /\[[^]]+\]/) == 0) {
                    next
                }

                vec = substr($0, RSTART + 1, RLENGTH - 2)
                nsplit = split(vec, parts, ",")
                if (nsplit < 3) {
                    next
                }

                yaw = trim(parts[3]) + 0.0
                pi = atan2(0, -1)

                if (n == 0) {
                    first = yaw
                    prev = yaw
                    unwrapped = yaw
                    min_u = yaw
                    max_u = yaw
                } else {
                    delta = yaw - prev
                    while (delta > pi) delta -= 2.0 * pi
                    while (delta < -pi) delta += 2.0 * pi
                    unwrapped += delta
                    prev = yaw
                    if (unwrapped < min_u) min_u = unwrapped
                    if (unwrapped > max_u) max_u = unwrapped
                }

                last = unwrapped
                n++
            }
            END {
                if (n < 2) {
                    print "status=FAIL"
                    print "reason=insufficient_tf_samples"
                    print "samples=" n
                    exit 3
                }

                span = max_u - min_u
                span_deg = span * 180.0 / atan2(0, -1)
                delta = last - first
                delta_deg = delta * 180.0 / atan2(0, -1)

                print "samples=" n
                printf("yaw_span_rad=%.6f\n", span)
                printf("yaw_span_deg=%.2f\n", span_deg)
                printf("yaw_delta_rad=%.6f\n", delta)
                printf("yaw_delta_deg=%.2f\n", delta_deg)
                printf("min_required_span_rad=%.6f\n", min_span + 0.0)

                if (span >= (min_span + 0.0)) {
                    print "status=PASS"
                    exit 0
                }

                print "status=FAIL"
                print "reason=yaw_span_below_threshold"
                exit 4
            }
        '
)"; then
    echo "Rotation verification failed."
    echo "${result:-no output}"
    exit 1
fi

echo "${result}"
