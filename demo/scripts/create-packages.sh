#!/bin/bash
#
# Create Gap Follower artifact packages for OTA demo
# This script creates tar.gz archives for each variant and updates stack JSON checksums
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_DIR="${SCRIPT_DIR}/.."
ARTIFACTS_DIR="${DEMO_DIR}/artifacts"
STACKS_DIR="${DEMO_DIR}/stacks"
VARIANTS_DIR="${DEMO_DIR}/variants"
GAP_FOLLOWER_SRC="${DEMO_DIR}/../../gap_follower"

echo "=== Creating Gap Follower Demo Packages ==="
echo "Script directory: ${SCRIPT_DIR}"
echo "Artifacts directory: ${ARTIFACTS_DIR}"
echo "Gap follower source: ${GAP_FOLLOWER_SRC}"
echo ""

# Ensure directories exist
mkdir -p "${ARTIFACTS_DIR}"

# Check if gap_follower source exists
if [ ! -d "${GAP_FOLLOWER_SRC}" ]; then
    echo "ERROR: Gap follower source not found at ${GAP_FOLLOWER_SRC}"
    exit 1
fi

# Function to create a variant package
create_package() {
    local variant=$1
    local version=$2

    echo "Creating ${variant} variant package (v${version})..."

    # Create temporary directory for packaging
    local temp_dir=$(mktemp -d)
    local package_name="gap_follower_${variant}"
    local archive_name="${package_name}.tar.gz"

    # Copy gap_follower source to src/ directory (standard colcon workspace layout)
    mkdir -p "${temp_dir}/src/gap_follower"
    cp -r "${GAP_FOLLOWER_SRC}/gap_follower" "${temp_dir}/src/gap_follower/"
    cp -r "${GAP_FOLLOWER_SRC}/config" "${temp_dir}/src/gap_follower/" 2>/dev/null || true
    cp -r "${GAP_FOLLOWER_SRC}/launch" "${temp_dir}/src/gap_follower/" 2>/dev/null || true
    cp -r "${GAP_FOLLOWER_SRC}/resource" "${temp_dir}/src/gap_follower/"
    cp -r "${GAP_FOLLOWER_SRC}/test" "${temp_dir}/src/gap_follower/" 2>/dev/null || true
    cp "${GAP_FOLLOWER_SRC}/package.xml" "${temp_dir}/src/gap_follower/"
    cp "${GAP_FOLLOWER_SRC}/setup.cfg" "${temp_dir}/src/gap_follower/"
    cp "${GAP_FOLLOWER_SRC}/setup.py" "${temp_dir}/src/gap_follower/"

    # Copy variant-specific run.sh to workspace root
    cp "${VARIANTS_DIR}/${variant}/run.sh" "${temp_dir}/run.sh"
    chmod +x "${temp_dir}/run.sh"

    # Create the archive (exclude build/install/log directories)
    tar -czf "${ARTIFACTS_DIR}/${archive_name}" -C "${temp_dir}" .

    # Calculate checksum
    local checksum=$(sha256sum "${ARTIFACTS_DIR}/${archive_name}" | cut -d' ' -f1)

    echo "  Archive: ${archive_name}"
    echo "  Checksum: ${checksum}"

    # Update stack JSON with checksum (replace PLACEHOLDER_CHECKSUM)
    local stack_file="${STACKS_DIR}/gap_follower_${variant}.json"
    if [ -f "${stack_file}" ]; then
        # Use sed to replace the checksum placeholder or existing checksum
        sed -i "s/\"checksum\": \"[^\"]*\"/\"checksum\": \"${checksum}\"/g" "${stack_file}"
        echo "  Updated: ${stack_file}"
    fi

    # Cleanup
    rm -rf "${temp_dir}"

    echo ""
}

# Create packages for each variant
create_package "conservative" "1.0.0"
create_package "balanced" "1.1.0"
create_package "aggressive" "1.2.0"
create_package "broken" "1.3.0"

echo "=== Package Creation Complete ==="
echo ""
echo "Artifacts created in: ${ARTIFACTS_DIR}"
ls -la "${ARTIFACTS_DIR}"
echo ""
echo "Stack definitions in: ${STACKS_DIR}"
ls -la "${STACKS_DIR}"
echo ""
echo "To serve artifacts, run:"
echo "  cd ${ARTIFACTS_DIR} && python3 -m http.server 9090"
