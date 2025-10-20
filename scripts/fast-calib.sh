#!/usr/bin/env bash
set -e

echo "Cloning FAST-Calib..."
if [ ! -d "FAST-Calib" ]; then
  git clone https://github.com/hku-mars/FAST-Calib.git
fi

PKG_XML="FAST-Calib/package.xml"
CMAKELISTS="FAST-Calib/CMakeLists.txt"

echo "Patching $PKG_XML..."

# Insert build_depend image_geometry if missing
if ! grep -q "<build_depend>image_geometry</build_depend>" "$PKG_XML"; then
  awk '/<build_depend>cv_bridge<\/build_depend>/ && !x {print; print "  <build_depend>image_geometry</build_depend>"; x=1; next}1' "$PKG_XML" > tmp && mv tmp "$PKG_XML"
fi

# Insert run_depend image_geometry if missing
if ! grep -q "<run_depend>image_geometry</run_depend>" "$PKG_XML"; then
  awk '/<run_depend>cv_bridge<\/run_depend>/ && !y {print; print "  <run_depend>image_geometry</run_depend>"; y=1; next}1' "$PKG_XML" > tmp && mv tmp "$PKG_XML"
fi

echo "Patching $CMAKELISTS..."

# Insert into find_package(...)
if ! grep -qE "find_package\(catkin REQUIRED COMPONENTS.*cv_bridge" "$CMAKELISTS"; then
  awk '
    /find_package\(catkin REQUIRED COMPONENTS/ {in_block=1}
    in_block && /^\)/ {
      print "  cv_bridge";
      print "  image_transport";
      print "  image_geometry";
      in_block=0;
    }
    {print}
  ' "$CMAKELISTS" > tmp && mv tmp "$CMAKELISTS"
fi

# Insert into catkin_package(...)
if ! grep -qE "catkin_package\(.*CATKIN_DEPENDS.*geometry_msgs" "$CMAKELISTS"; then
  awk '
    /catkin_package\(/ {in_cp=1}
    in_cp && /^\)/ {
      print "  geometry_msgs";
      print "  cv_bridge";
      print "  image_transport";
      print "  image_geometry";
      in_cp=0;
    }
    {print}
  ' "$CMAKELISTS" > tmp && mv tmp "$CMAKELISTS"
fi

echo "✅ FAST-Calib successfully patched (portable sed-safe version)."
