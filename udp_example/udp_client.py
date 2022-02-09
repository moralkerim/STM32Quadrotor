import socket
import struct


## GONDERILECEK ORNEK BIR PAKET ##
baslangic_biti = 0xABCD
takim_numarasi = 37
saat = 12
dakika = 5

saniye = 30
salise = 4
enlem = 41.0112
boylam = 28.942
irtifa = 500
dikilme = 20.3
yonelme = 10.5
yatis = 40.8
hiz = 5.4
yakit = 70

## SERVER BAGLANTISI ##
address = ('192.168.1.45', 9000) ## SERVER IP ADRESS + PORT
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.connect(address)


while True:
	## AYNI PAKETI SONSUZ DONGUDE SERVER TARAFINA GONDERIYOURUZ
	## GONDERILECEK PAKETI AYARLIYORUZ
    binary = struct.pack("<HHBBBBddfffffH",baslangic_biti, takim_numarasi,saat,dakika,saniye,salise,enlem, boylam, irtifa, dikilme, yonelme, yatis, hiz, yakit )
	##GONDERME FONKSIYONU
    sock.sendall(binary)
