clear
# Many of the below items are from
#https://tweakhound.com/2015/12/09/tweaking-windows-10/
   
#cd ~\Desktop
#mkdir "God Mode.{ED7BA470-8E54-465E-825C-99712043E01C}"
   
function Change-Service ($servicename, $serviceaction, $servicestatuptype)
{
    if (Get-Service $servicename -ErrorAction SilentlyContinue)
        {
            $ServiceDetail=$(Get-Service -Displayname $servicename)
            Write-Output "$servicename ($($ServiceDetail.Name))"
            switch ($serviceaction)
                {
                    "start" {
                        if ($ServiceDetail.Status -eq "Stopped")
                        {
                            Write-Output "    Starting"
                            Start-Service $servicename
                        }
                        else
                        {
                            Write-Output "    Is not Stopped"
                        }
                    }
                    default {
                        if ($ServiceDetail.Status -ne "Stopped")
                        {
                            Write-Output "    Stoping"
                            Stop-Service $servicename
                        }
                        else
                        {
                            Write-Output "    Is already Stopped"
                        }
                    }
                }
   
            if ($ServiceDetail.StartType -ne $servicestatuptype)
            {
                Write-Output "    Setting to $servicestatuptype"
                Set-Service $ServiceDetail.Name -StartupType $servicestatuptype
            }
            else
            {
                Write-Output "    Is already set to $servicestatuptype"
            }
        }
}
   
Change-Service "Block Level Backup Engine Service" stop manual
Change-Service "Certificate Propagation" stop manual
Change-Service "Distributed Link Tracking Client" stop manual
Change-Service "dmwappushsvc" stop manual
Change-Service "HomeGroup Listener" stop manual
Change-Service "HomeGroup Provider" stop manual
Change-Service "IP Helper" stop manual
Change-Service "Portable Device Enumerator Service" stop manual
Change-Service "Remote Registry" stop manual
Change-Service "Secondary Logon" stop manual
Change-Service "Software Protection" stop manual
Change-Service "SSDP Discovery" stop manual
Change-Service "TCP/IP NetBIOS Helper" stop manual
Change-Service "Touch Keyboard and Handwriting Panel Service" stop manual
Change-Service "Windows Media Player Network Sharing Service" stop manual
Change-Service "Windows Time" stop manual
Change-Service AeLookupSvc stop manual
Change-Service ehRecvr stop manual
Change-Service ehSched stop manual
Change-Service ProtectedStorage stop manual
Change-Service Themes stop manual
Change-Service UxSms stop manual
   
# List all apps
$Appx = Get-AppxPackage -AllUsers | select name, packagefullname  | sort name
$appx
  
# Disable Cloud Content
$item = New-Item "REGISTRY::HKEY_LOCAL_MACHINE\SOFTWARE\Policies\Microsoft\Windows\CloudContent"  -Force
Set-ItemProperty $item.PSPath -Name DisableWindowsConsumerFeatures -Type DWord -Value 1
 
function Remove-WindowsApp ($appName)
{
Get-AppxPackage | where-object {$_.name -like "*$appName*"} | Remove-AppxPackage
Get-AppxPackage -AllUsers | where-object {$_.name -like "*$appName*"} | Remove-AppxPackage
}
 
Remove-WindowsApp 3d
Remove-WindowsApp 3dbuilder
# Remove-WindowsApp alarms
Remove-WindowsApp appconnector
Remove-WindowsApp appinstaller
Remove-WindowsApp AutodeskSketchBook
Remove-WindowsApp bing
Remove-WindowsApp bingfinance
Remove-WindowsApp bingnews
Remove-WindowsApp bingsports
Remove-WindowsApp bingweather
Remove-WindowsApp BubbleWitch3Saga
# Remove-WindowsApp calculator
Remove-WindowsApp camera
Remove-WindowsApp candycrushsodasaga
Remove-WindowsApp commsphone
Remove-WindowsApp communi
Remove-WindowsApp communicationsapps
Remove-WindowsApp connectivitystore
Remove-WindowsApp DisneyMagicKingdoms
Remove-WindowsApp DrawboardPDF
Remove-WindowsApp Duolingo
Remove-WindowsApp Eclipse
Remove-WindowsApp FarmVille
Remove-WindowsApp feedback
Remove-WindowsApp FLipboard
Remove-WindowsApp FreshPaint
Remove-WindowsApp getstarted
Remove-WindowsApp holographic
Remove-WindowsApp king.com
Remove-WindowsApp Mahjong
# Remove-WindowsApp maps
Remove-WindowsApp MarchofEmpires
Remove-WindowsApp messaging
Remove-WindowsApp MicrosoftSudoku
Remove-WindowsApp mspaint
Remove-WindowsApp NetworkSpeedTest
Remove-WindowsApp NYTCrossword
Remove-WindowsApp officehub
Remove-WindowsApp oneconnect
Remove-WindowsApp onenote
Remove-WindowsApp pandora
# Remove-WindowsApp people
Remove-WindowsApp phone
# Remove-WindowsApp photo
# Remove-WindowsApp photos
Remove-WindowsApp skypeapp
Remove-WindowsApp solit
Remove-WindowsApp solitaire
Remove-WindowsApp soundrec
Remove-WindowsApp soundrecorder
Remove-WindowsApp Spotify
Remove-WindowsApp sticky
Remove-WindowsApp sway
Remove-WindowsApp twitter
Remove-WindowsApp wallet
Remove-WindowsApp windowscommunicationsapps
Remove-WindowsApp windowsphone
#Remove-WindowsApp windowsstore
Remove-WindowsApp witch 
Remove-WindowsApp Wunderlist
Remove-WindowsApp xbox
Remove-WindowsApp zune
Remove-WindowsApp zunemusic
Remove-WindowsApp zunevideo
   
# To Reinstall all of the above run the below
# Get-AppXPackage -AllUsers | Foreach {Add-AppxPackage -DisableDevelopmentMode -Register "$($_.InstallLocation)\\AppXManifest.xml"}