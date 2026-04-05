import zipfile, io, re, os, sys

sys.stdout.reconfigure(encoding='utf-8')

base = r'c:\Scooter_Charger\Scooter_Charger\TC_Charger_Documentation'

xlsx_path = os.path.join(base, 'TC Standard OBC CAN Protocal .xlsx')
with open(xlsx_path, 'rb') as f:
    data = f.read()

zf = zipfile.ZipFile(io.BytesIO(data))

strings = []
if 'xl/sharedStrings.xml' in zf.namelist():
    ss_xml = zf.read('xl/sharedStrings.xml').decode('utf-8', errors='ignore')
    strings = re.findall(r'<t[^>]*>([^<]*)</t>', ss_xml)
    print("=== Shared Strings ===")
    for i, s in enumerate(strings):
        print(f"  [{i}] {s}")

for name in zf.namelist():
    if name.startswith('xl/worksheets/') and name.endswith('.xml'):
        sheet_xml = zf.read(name).decode('utf-8', errors='ignore')
        print(f"\n=== {name} ===")
        rows = re.findall(r'<row[^>]*>(.*?)</row>', sheet_xml, re.DOTALL)
        for row in rows:
            cells = re.findall(r'<c r="([^"]+)"(?:[^>]*t="([^"]*)")?[^>]*>.*?<v>([^<]*)</v>', row)
            row_vals = []
            for ref, typ, val in cells:
                if typ == 's':
                    idx = int(val)
                    val = strings[idx] if idx < len(strings) else f'#{idx}'
                row_vals.append(f"{ref}={val}")
            if row_vals:
                print("  " + " | ".join(row_vals))
    try:
        import zlib
        text = zlib.decompress(s)
    except Exception:
        text = s
    # pull out text between BT/ET
    chunks = re.findall(rb'\(([^\)]{2,200})\)', text)
    for c in chunks:
        try:
            decoded = c.decode('latin-1').strip()
            if decoded and any(ch.isalpha() for ch in decoded):
                print(decoded)
        except Exception:
            pass
