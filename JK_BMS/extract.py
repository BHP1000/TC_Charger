import pymupdf
doc = pymupdf.open(r'c:\Scooter_Charger\Scooter_Charger\JK_BMS\JK_BMS_RS485_Modbus.pdf')
out = open(r'c:\Scooter_Charger\Scooter_Charger\JK_BMS\extracted.txt', 'w', encoding='utf-8', errors='replace')
for i in range(doc.page_count):
    out.write('=== PAGE ' + str(i+1) + ' ===\n')
    out.write(doc[i].get_text())
    out.write('\n')
out.close()
doc.close()
