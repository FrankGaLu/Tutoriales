#!/usr/bin/env bash
# install_ros2.sh
# Ejecutable, con checklist y barra de progreso. Uso: ./install_ros2.sh [--dry-run]

set -euo pipefail
IFS=$'\n\t'

DRY_RUN=0
if [[ ${1:-} == "--dry-run" ]]; then
	DRY_RUN=1
fi

TOTAL_STEPS=8
LABELS=(
	"Actualizar índices y instalar curl"
	"Obtener versión más reciente de ros-apt-source"
	"Descargar paquete ros2-apt-source"
	"Instalar paquete ros2-apt-source"
	"Actualizar índices APT (después del source)"
	"Actualizar paquetes del sistema"
	"Instalar ros-jazzy-desktop-full"
	"Instalar colcon extensions (python3)"
)

CMDS=(
	"sudo apt update && sudo apt install -y curl"
	"echo \"Obtener versión (se calculará antes de ejecutar)\""
	"echo \"Descargar paquete (se usará URL calculada)\""
	"sudo dpkg -i /tmp/ros2-apt-source.deb"
	"sudo apt update"
	"sudo DEBIAN_FRONTEND=noninteractive apt -y upgrade"
	"sudo DEBIAN_FRONTEND=noninteractive apt -y install ros-jazzy-desktop-full"
	"sudo DEBIAN_FRONTEND=noninteractive apt -y install python3-colcon-common-extensions"
)

# Pre-compute ROS apt source version and download URL so commands can use them
ROS_APT_SOURCE_VERSION=""
DOWNLOAD_URL=""
if command -v curl >/dev/null 2>&1; then
	# Try to get the latest tag name from GitHub
	ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}' || true)
fi
if [[ -f /etc/os-release ]]; then
	# shellcheck disable=SC1091
	. /etc/os-release
fi
if [[ -n "$ROS_APT_SOURCE_VERSION" && -n "${VERSION_CODENAME:-}" ]]; then
	DOWNLOAD_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
else
	DOWNLOAD_URL=""
fi

# Insert the computed actions into CMDS for steps 2 and 3
if [[ -n "$DOWNLOAD_URL" ]]; then
	# step index 2 (0-based) is the download step
	CMDS[2]="curl -L -o /tmp/ros2-apt-source.deb \"$DOWNLOAD_URL\""
else
	CMDS[2]="echo \"DOWNLOAD_URL no disponible; revisa la conexión de red o el cálculo de VERSION_CODENAME\""
fi
BAR_WIDTH=40

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
RESET='\033[0m'

print_header() {
	echo
	printf "${BOLD}${BLUE}==============================\n"
	printf " ROS2 installer (script)\n"
	printf "==============================${RESET}\n\n"
}

draw_progress() {
	local idx=$1
	local total=$2
	local label="$3"
	local pct=$(( (idx*100)/total ))
	local filled=$(( (pct*BAR_WIDTH)/100 ))
	local empty=$(( BAR_WIDTH-filled ))
	printf "[%3d%%] [" "$pct"
	for ((i=0;i<filled;i++)); do printf "%b#%b" "$GREEN" "$RESET"; done
	for ((i=0;i<empty;i++)); do printf "%b-%b" "$BLUE" "$RESET"; done
	printf "] %s\n" "$label"
}

print_checklist() {
	local cur=$1
	for idx in $(seq 0 $((TOTAL_STEPS-1))); do
		local mark=" "
		local label_color=""
		if [[ $idx -lt $cur ]]; then
			mark="x"
			label_color="$GREEN"
		elif [[ $idx -eq $cur ]]; then
			mark=">"
			label_color="$YELLOW"
		else
			label_color="$RESET"
		fi
		printf "[%b%s%b] %2d/%d %b%s%b\n" "$BOLD" "$mark" "$RESET" $((idx+1)) $TOTAL_STEPS "$label_color" "${LABELS[idx]}" "$RESET"
	done
}

run_step() {
	local i=$1
	local cmd="${CMDS[i]}"
	local label="${LABELS[i]}"

	print_checklist $i
	draw_progress $i $TOTAL_STEPS "$label"

	if [[ $DRY_RUN -eq 1 ]]; then
	printf "%b[DRY-RUN]%b Ejecutaría: %s\n" "$YELLOW" "$RESET" "$cmd"
		sleep 1
		return 0
	fi

	# Run the command in a subshell to avoid polluting environment except where intended
		if bash -c "$cmd"; then
			printf "%b[OK]%b %s\n" "$GREEN" "$RESET" "$label"
		return 0
	else
			printf "%b[FAIL]%b %s\n" "$RED" "$RESET" "$label"
		return 1
	fi
}

main() {
	print_header
	echo "Modo: $([[ $DRY_RUN -eq 1 ]] && echo 'DRY-RUN' || echo 'EJECUTANDO')"
	echo

	for i in $(seq 0 $((TOTAL_STEPS-1))); do
		if ! run_step $i; then
			echo
			echo "Installation failed on step $((i+1)): ${LABELS[i]}"
			echo "Revisa la salida anterior para detalles."
			exit 2
		fi
		echo
	done

		# final state: mark all done
		print_checklist $TOTAL_STEPS
		draw_progress $TOTAL_STEPS $TOTAL_STEPS "Todos los pasos completados"
		echo
		printf "%b¡Listo!%b\n" "$BOLD$GREEN" "$RESET"
}

main
