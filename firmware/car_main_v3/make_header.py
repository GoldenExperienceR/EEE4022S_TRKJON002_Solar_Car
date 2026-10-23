# Rebuild dashboard_gz.h after editing page/dashboard.html:  python make_header.py
import gzip, pathlib
here = pathlib.Path(__file__).parent
gz = gzip.compress((here / "page" / "dashboard.html").read_bytes(), 9, mtime=0)
rows = [",".join("0x%02x" % b for b in gz[k:k + 24]) for k in range(0, len(gz), 24)]
(here / "dashboard_gz.h").write_text(
    "// Generated from page/dashboard.html (gzip). Rebuild with make_header.py after editing the page.\n"
    "#pragma once\n#include <Arduino.h>\n"
    f"const size_t DASH_GZ_LEN = {len(gz)};\n"
    "const uint8_t DASH_GZ[] PROGMEM = {\n" + ",\n".join(rows) + "\n};\n")
print("dashboard_gz.h:", len(gz), "bytes gzipped")
