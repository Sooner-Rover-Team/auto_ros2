#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  scripts/get_navsat_yaw_offset.sh --heading-deg <deg> [options]

Computes a recommended /navsat_transform_node yaw_offset from live IMU data.

Heading convention for --heading-deg:
  0 = North, 90 = East, 180 = South, 270 = West.

Options:
  --topic <topic>       IMU topic to sample (default: /unilidar/imu)
  --duration <seconds>  Sampling time (default: 10)
  --node <name>         Node used by --apply (default: /navsat_transform_node)
  --apply               Apply computed yaw_offset live with ros2 param set
  --no-source           Skip sourcing install/setup.bash
  -h, --help            Show this help
EOF
}

topic="/unilidar/imu"
duration="10"
node="/navsat_transform_node"
apply_now=0
auto_source=1
heading_deg=""

while [[ $# -gt 0 ]]; do
    case "$1" in
    --heading-deg)
        heading_deg="${2:-}"
        shift 2
        ;;
    --topic)
        topic="${2:-}"
        shift 2
        ;;
    --duration)
        duration="${2:-}"
        shift 2
        ;;
    --node)
        node="${2:-}"
        shift 2
        ;;
    --apply)
        apply_now=1
        shift
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

if [[ -z "${heading_deg}" ]]; then
    echo "Missing required --heading-deg <deg>" >&2
    usage >&2
    exit 2
fi

if ! [[ "${heading_deg}" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
    echo "--heading-deg must be numeric" >&2
    exit 2
fi

if ! [[ "${duration}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "--duration must be numeric" >&2
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

awk_program='
function wrap_deg(a) { while (a > 180.0) a -= 360.0; while (a <= -180.0) a += 360.0; return a }
function wrap_rad(a) {
    pi = atan2(0, -1)
    while (a > pi) a -= 2.0 * pi
    while (a <= -pi) a += 2.0 * pi
    return a
}
BEGIN {
    pi = atan2(0, -1)
    n = 0
    sum_s = 0.0
    sum_c = 0.0
    sum_wz = 0.0
}
NF >= 19 {
    x = $4 + 0.0
    y = $5 + 0.0
    z = $6 + 0.0
    w = $7 + 0.0
    yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    sum_s += sin(yaw)
    sum_c += cos(yaw)
    sum_wz += ($19 + 0.0)
    n++
}
END {
    if (n < 1) {
        exit 2
    }

    mean_yaw = atan2(sum_s / n, sum_c / n)
    mean_yaw_deg = mean_yaw * 180.0 / pi
    expected_yaw_deg = wrap_deg(90.0 - (heading_deg + 0.0))
    offset_deg = wrap_deg(expected_yaw_deg - mean_yaw_deg)
    offset_rad = wrap_rad(offset_deg * pi / 180.0)

    R = sqrt((sum_c / n) * (sum_c / n) + (sum_s / n) * (sum_s / n))
    if (R <= 0.0) {
        yaw_spread_deg = 180.0
    } else {
        yaw_spread_deg = sqrt(-2.0 * log(R)) * 180.0 / pi
    }

    printf("samples=%d\n", n)
    printf("mean_imu_yaw_deg=%.6f\n", mean_yaw_deg)
    printf("yaw_spread_deg=%.6f\n", yaw_spread_deg)
    printf("expected_yaw_deg=%.6f\n", expected_yaw_deg)
    printf("recommended_yaw_offset_deg=%.6f\n", offset_deg)
    printf("recommended_yaw_offset_rad=%.6f\n", offset_rad)
    printf("mean_gyro_z_rad_s=%.6f\n", sum_wz / n)
}
'

if ! stats="$(
    {
        timeout "${duration}" ros2 topic echo "${topic}" --csv 2>/dev/null || true
    } | awk -F, -v heading_deg="${heading_deg}" "${awk_program}"
)"; then
    echo "No IMU samples received from ${topic} in ${duration}s." >&2
    exit 1
fi

echo "${stats}"
offset_rad="$(awk -F= '/^recommended_yaw_offset_rad=/{print $2}' <<<"${stats}")"

if [[ -z "${offset_rad}" ]]; then
    echo "Failed to compute recommended_yaw_offset_rad." >&2
    exit 1
fi

echo "apply_cmd=ros2 param set ${node} yaw_offset ${offset_rad}"

if [[ "${apply_now}" -eq 1 ]]; then
    ros2 param set "${node}" yaw_offset "${offset_rad}"
fi
