Get-ChildItem -Path "E:\Projects\LilyGo-LoRa-Series\lib\RadioLib\src\modules\LR11x0" -Recurse -Include *.h, *.cpp | 
    Get-Content | Set-Content -Path "E:\Projects\micropython-lora\merged_code.txt"