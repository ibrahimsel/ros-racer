#!/bin/bash
# =============================================================================
# Generate Artifact Archives for Muto Demo
# =============================================================================
# This script creates the tar.gz artifacts from the variant directories
# and updates the stack JSON files with the correct checksums.
#
# Run this before starting the demo to ensure artifacts are up to date.
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_DIR="$(dirname "$SCRIPT_DIR")"
VARIANTS_DIR="$DEMO_DIR/variants"
ARTIFACTS_DIR="$DEMO_DIR/artifacts"
STACKS_DIR="$DEMO_DIR/stacks"
SYMPHONY_DIR="$(dirname "$DEMO_DIR")/symphony"

# Colors
GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}Generating Muto demo artifacts...${NC}"

# Create artifacts directory if it doesn't exist
mkdir -p "$ARTIFACTS_DIR"

# Function to create tar and update JSON files
create_artifact() {
    local variant=$1
    local tar_name="gap_follower_${variant}.tar.gz"

    echo -n "  Creating $tar_name... "

    # Create tar.gz from variant directory
    tar -czf "$ARTIFACTS_DIR/$tar_name" -C "$VARIANTS_DIR/$variant" .

    # Calculate checksum
    local checksum=$(sha256sum "$ARTIFACTS_DIR/$tar_name" | cut -d' ' -f1)

    # Update demo/stacks JSON (uses artifact-server hostname)
    local stacks_json="$STACKS_DIR/gap_follower_${variant}.json"
    if [ -f "$stacks_json" ]; then
        # Use sed to update checksum in place
        sed -i "s/\"checksum\": \"[a-f0-9]*\"/\"checksum\": \"$checksum\"/" "$stacks_json"
    fi

    # Update symphony JSON (uses 172.17.0.1 for docker host)
    local symphony_json="$SYMPHONY_DIR/gap-follower-${variant}.json"
    if [ -f "$symphony_json" ]; then
        sed -i "s/\"checksum\": \"[a-f0-9]*\"/\"checksum\": \"$checksum\"/" "$symphony_json"
    fi

    echo -e "${GREEN}done${NC} (checksum: ${checksum:0:16}...)"
}

# Generate all variants
create_artifact "conservative"
create_artifact "balanced"
create_artifact "aggressive"
create_artifact "broken"

echo ""
echo -e "${GREEN}All artifacts generated successfully!${NC}"
echo ""
echo "Artifacts location: $ARTIFACTS_DIR"
ls -lh "$ARTIFACTS_DIR"/*.tar.gz
