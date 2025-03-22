c
    # Enable WiFi
    Enable-NetAdapter -Name "Wi-Fi" -Confirm:$false
}
else
{
    # Disable WiFi
    Disable-NetAdapter -Name "Wi-Fi" -Confirm:$false
}

Get-NetAdapter | Format-Table

ipconfig /flushdns
ipconfig /renew

pause