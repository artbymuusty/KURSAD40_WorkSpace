from core.mission.interlock import PayloadInterlock

def check_gorev3_precondition(interlock: PayloadInterlock) -> bool:
    """
    Görev 3 Rapor Bölüm 3: Görev 3 YALNIZCA
    payload_1_released == True VE payload_2_released == True olduğunda başlar.
    interlock.both_released() bunu zaten sağlıyor; bu fonksiyon yalnızca
    Görev 3'e özgü, açıkça isimlendirilmiş bir giriş kapısıdır (okunabilirlik için).
    
    Eğer bu False dönerse:
    Görev 3 orkestratörü (Prompt 33) hata mesajı loglar, görev başlatılmaz ve
    Görev 3'e ASLA girilmez.
    """
    return interlock.both_released()
