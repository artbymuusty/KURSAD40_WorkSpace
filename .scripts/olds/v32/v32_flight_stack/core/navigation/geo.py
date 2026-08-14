"""
Görev 2 Rapor (operatör revizyonu, 2026-08-13, "Mission Lifecycle" görev
yeniden yapılandırması): Search tamamlandığında araç ikinci hedefin
yakınında olabilir -- Payload Mission 1/2, kaydedilmiş bir GPS konumuna
(Mavi Altıgen / Kırmızı Üçgen) geri dönmeyi gerektirir. Bu, her
gorev3_*.py fazında zaten kabul edilmiş ("GPS->NED dönüşümü basitleştirilmiştir")
bir boşluktu; bu modül o boşluğu gerçek (ama kasıtlı olarak minimal) bir
düzlem-dünya (flat-earth/equirectangular) yaklaşıklığıyla kapatır.

Bilinçli sınırlama: Bu, tam bir jeodezi kütüphanesi DEĞİLDİR -- yarışma
alanı ölçeğinde (birkaç yüz metre) doğruluğu %0.5'in altındadır, ancak
kilometrelerce mesafede yanlışlaşır. Yeni bir navigasyon çerçevesi icat
etmek yerine mevcut Offboard ilkeliyle (goto_position_ned) birlikte
kullanılmak üzere tasarlanmıştır.
"""
import math

_EARTH_RADIUS_M = 6371000.0


def gps_to_ned_delta(from_lat: float, from_lon: float, to_lat: float, to_lon: float) -> tuple[float, float]:
    """`from_` konumundan `to` konumuna düz-dünya (north_m, east_m) delta'sı.
    goto_position_ned()'in beklediği NED referans çerçevesiyle doğrudan
    uyumludur (north_m, east_m göreceli, mevcut konumdan)."""
    d_lat = math.radians(to_lat - from_lat)
    d_lon = math.radians(to_lon - from_lon)
    mean_lat = math.radians((from_lat + to_lat) / 2.0)
    north_m = d_lat * _EARTH_RADIUS_M
    east_m = d_lon * _EARTH_RADIUS_M * math.cos(mean_lat)
    return north_m, east_m


def haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """İki GPS koordinatı arasındaki büyük çember mesafesi (metre) --
    yakınsama kontrolü için (gps_to_ned_delta'nın düz-dünya yaklaşıklığından
    bağımsız, standart formül)."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = math.sin(d_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return _EARTH_RADIUS_M * c
