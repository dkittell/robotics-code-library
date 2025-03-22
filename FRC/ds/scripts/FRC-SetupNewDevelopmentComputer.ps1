<#
    Prerequisites:
    1. Chocolatey - https://chocolatey.org/install
    Set-ExecutionPolicy Bypass -Scope Process -Force;
    [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072;
    iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))
#>

#region Variables
$sGitPath = "c:\git"
#endregion Variables

#region Functions
function Is-Installed ($program) {

  $x86 = ((Get-ChildItem "HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall") |
    Where-Object { $_.GetValue("DisplayName") -like "*$program*" }).Length -gt 0;

  $x64 = ((Get-ChildItem "HKLM:\Software\Wow6432Node\Microsoft\Windows\CurrentVersion\Uninstall") |
    Where-Object { $_.GetValue("DisplayName") -like "*$program*" }).Length -gt 0;

  return $x86 -or $x64;
}

function Is-Enabled ($feature) {

  $status = Get-WindowsOptionalFeature -Online | Where-Object { $_.FeatureName -eq $feature }

  return $status.State;
}

function Enable-Feature ($feature){
  if ($(Is-Enabled $feature) -eq $null) {
    Write-Output "Enabling $feature"
    Enable-WindowsOptionalFeature -Online -FeatureName $feature
  }
  else
  {
    Write-Output "$feature Already Enabled"
  }
}

function Choco-Installed ($program) {
  $result = $(choco list -lo | Where-Object { $_.ToLower().StartsWith($program.ToLower()) })

  if ($result) {
    Write-Output "$result Installed"
  }
  else
  {
    Write-Output "Installing $result"
    choco install $program -y
  }
}


#region Require confirmation to delete (send items to recycle bin)
$registryPath = "HKCU:\Software\Microsoft\Windows\CurrentVersion\Policies\Explorer"
$Name = "ConfirmFileDelete"
$value = "1"
IF(!(Test-Path $registryPath))
{
New-Item -Path $registryPath -Force | Out-Null
New-ItemProperty -Path $registryPath -Name $name -Value $value -PropertyType DWORD -Force | Out-Null
}
#endregion Require confirmation to delete (send items to recycle bin)

#region Git
if ($(Is-Installed "Git") -eq $null -or $(Is-Installed "Git") -eq $false) {
  Write-Output "Installing Git"
  choco install Git -y
}
else
{
  Write-Output "Git Already Installed"
}
#endregion Git

#region winscp
Choco-Installed winscp
Choco-Installed putty
#endregion winscp

#region wget
Choco-Installed wget
#endregion wget

# Set taskbar left align
reg add hkcu\software\microsoft\windows\currentversion\explorer\advanced /v TaskbarAl /t REG_DWORD /f /d 0

powershell.exe -noprofile -executionpolicy bypass -file CustomizeTaskbar.ps1 -RemoveWidgets -StartMorePins –MoveStartLeft -RemoveTaskView -RemoveChat -RemoveSearch -RunForExistingUsers