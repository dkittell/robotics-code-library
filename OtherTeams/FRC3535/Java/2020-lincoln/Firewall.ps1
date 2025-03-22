# Disable Firewall
# Set-NetFirewallProfile -Profile Domain,Public,Private -Enabled False
  
# Enable Firewall
Set-NetFirewallProfile -Profile Domain,Public,Private -Enabled True

Get-NetFirewallProfile -Profile Domain,Public,Private | FT -AutoSize Name, Enabled

ipconfig /flushdns
ipconfig /release
ipconfig /renew

# RESET FRC Firewall Rules
Remove-NetFirewallRule -DisplayName 'FRC - *'
  
New-NetFirewallRule -DisplayName 'FRC - VS Code' -Profile @('Domain','Private','Public') -Direction Outbound -RemoteAddress LocalSubnet -Action Allow -Program "C:\Program Files\Microsoft VS Code\Code.exe"
New-NetFirewallRule -DisplayName 'FRC - Drive Station' -Profile @('Domain','Private','Public') -Direction Outbound -RemoteAddress LocalSubnet -Action Allow -Program "C:\program files (x86)\national instruments\vision\nivissvr.exe"
New-NetFirewallRule -DisplayName 'FRC - Drive Station' -Profile @('Domain','Private','Public') -Direction Outbound -RemoteAddress LocalSubnet -Action Allow -Program "C:\program files (x86)\frc driver station\driverstation.exe"
New-NetFirewallRule -DisplayName 'FRC - roboRio Imaging Tool' -Profile @('Domain','Private','Public') -Direction Outbound -RemoteAddress LocalSubnet -Action Allow -Program "C:\program files (x86)\national instruments\labview 2018\project\roborio tool\roborio_imagingtool.exe"
New-NetFirewallRule -DisplayName 'FRC - Deploy Code - TCP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol TCP -LocalPort 1741-1742
New-NetFirewallRule -DisplayName 'FRC - Deploy Code - TCP' -Profile @('Domain','Private','Public') -Direction OUtbound -Action Allow -Protocol TCP -LocalPort 1741-1742
New-NetFirewallRule -DisplayName 'FRC - Deploy Code - TCP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol TCP -LocalPort 1577
New-NetFirewallRule -DisplayName 'FRC - Deploy Code - TCP' -Profile @('Domain','Private','Public') -Direction OUtbound -Action Allow -Protocol TCP -LocalPort 1577
New-NetFirewallRule -DisplayName 'FRC - Deploy Code - TCP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol TCP -LocalPort 3580
New-NetFirewallRule -DisplayName 'FRC - Deploy Code - TCP' -Profile @('Domain','Private','Public') -Direction OUtbound -Action Allow -Protocol TCP -LocalPort 3580
New-NetFirewallRule -DisplayName 'FRC - Camera Data - roboRio to Drive Station - TCP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol TCP -LocalPort 1180-1190
New-NetFirewallRule -DisplayName 'FRC - SmartDashboard - TCP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol TCP -LocalPort 1735
New-NetFirewallRule -DisplayName 'FRC - Camera HTTP - TCP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol TCP -LocalPort 80
New-NetFirewallRule -DisplayName 'FRC - Camera HTTPS - TCP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol TCP -LocalPort 443
New-NetFirewallRule -DisplayName 'FRC - Real-Time Stream - TCP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol TCP -LocalPort 554
New-NetFirewallRule -DisplayName 'FRC - CTRE Diagnostics Server - TCP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol TCP -LocalPort 1250
New-NetFirewallRule -DisplayName 'FRC - Team Use - TCP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol TCP -LocalPort 5800-5810
New-NetFirewallRule -DisplayName 'FRC - Camera Data - roboRio to Drive Station - TCP' -Profile @('Domain','Private','Public') -Direction Outbound -Action Allow -Protocol TCP -LocalPort 1180-1190
New-NetFirewallRule -DisplayName 'FRC - SmartDashboard - TCP' -Profile @('Domain','Private','Public') -Direction Outbound -Action Allow -Protocol TCP -LocalPort 1735
New-NetFirewallRule -DisplayName 'FRC - Camera HTTP - TCP' -Profile @('Domain','Private','Public') -Direction Outbound -Action Allow -Protocol TCP -LocalPort 80
New-NetFirewallRule -DisplayName 'FRC - Camera HTTPS - TCP' -Profile @('Domain','Private','Public') -Direction Outbound -Action Allow -Protocol TCP -LocalPort 443
New-NetFirewallRule -DisplayName 'FRC - Real-Time Stream - TCP' -Profile @('Domain','Private','Public') -Direction Outbound -Action Allow -Protocol TCP -LocalPort 554
New-NetFirewallRule -DisplayName 'FRC - CTRE Diagnostics Server - TCP' -Profile @('Domain','Private','Public') -Direction Outbound -Action Allow -Protocol TCP -LocalPort 1250
New-NetFirewallRule -DisplayName 'FRC - Team Use - TCP' -Profile @('Domain','Private','Public') -Direction Outbound -Action Allow -Protocol TCP -LocalPort 5800-5810
New-NetFirewallRule -DisplayName 'FRC - Camera Data - roboRio to Drive Station - UDP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol UDP -LocalPort 1180-1190
New-NetFirewallRule -DisplayName 'FRC - Dashboard-to-Robot - UDP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol UDP -LocalPort 1130
New-NetFirewallRule -DisplayName 'FRC - Robot-to-Dashboard - UDP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol UDP -LocalPort 1140
New-NetFirewallRule -DisplayName 'FRC - Real-Time Stream - UDP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol UDP -LocalPort 554
New-NetFirewallRule -DisplayName 'FRC - CTRE Diagnostics Server - UDP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol UDP -LocalPort 1250
New-NetFirewallRule -DisplayName 'FRC - Team Use - UDP' -Profile @('Domain','Private','Public') -Direction Inbound -Action Allow -Protocol UDP -LocalPort 5800-5810
New-NetFirewallRule -DisplayName 'FRC - Camera Data - roboRio to Drive Station - UDP' -Profile @('Domain','Private','Public') -Direction Outbound -Action Allow -Protocol UDP -LocalPort 1180-1190
New-NetFirewallRule -DisplayName 'FRC - Dashboard-to-Robot - UDP' -Profile @('Domain','Private','Public') -Direction Outbound -Action Allow -Protocol UDP -LocalPort 1130
New-NetFirewallRule -DisplayName 'FRC - Robot-to-Dashboard - UDP' -Profile @('Domain','Private','Public') -Direction Outbound -Action Allow -Protocol UDP -LocalPort 1140
New-NetFirewallRule -DisplayName 'FRC - Real-Time Stream - UDP' -Profile @('Domain','Private','Public') -Direction Outbound -Action Allow -Protocol UDP -LocalPort 554
New-NetFirewallRule -DisplayName 'FRC - CTRE Diagnostics Server - UDP' -Profile @('Domain','Private','Public') -Direction Outbound -Action Allow -Protocol UDP -LocalPort 1250
New-NetFirewallRule -DisplayName 'FRC - Team Use - UDP' -Profile @('Domain','Private','Public') -Direction Outbound -Action Allow -Protocol UDP -LocalPort 5800-5810
#endregion Firewall
