param(
      [Parameter(Mandatory = $False)]       
        [boolean] $enable = $False
)

if ($enable)
{
    # Enable Firewall
    Set-NetFirewallProfile -Profile Domain,Public,Private -Enabled True
}
else
{
    # Disable Firewall
    Set-NetFirewallProfile -Profile Domain,Public,Private -Enabled False -Verbose
}

Get-NetFirewallProfile -Profile Domain,Public,Private | FT -AutoSize Name, Enabled

pause