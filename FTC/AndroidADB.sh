# brew install android-platform-tools

adb devices

# for f in *.apk ; do adb install "$f" ; done

# pm list packages | grep -i 'talk'

# adb install ../apk/BarcodeScanner.apk
# adb install ../apk/APKExport.apk

adb shell

pm list packages | grep -i 'ftc'
pm uninstall com.qualcomm.ftcdriverstation
exit

adb install /Users/dkittell/Downloads/ftc.ds.9.1.apk
# adb install ~/repo/hostgator/public_html/downloads/apk/APK\ Export\ 3.2.4.apk

adb shell pm disable-user --user 0 cn.zte.music
adb shell pm disable-user --user 0 cn.zte.recorder
adb shell pm disable-user --user 0 com.coremobility.app.vnotes
adb shell pm disable-user --user 0 com.dreamgames.royalmatch
adb shell pm disable-user --user 0 com.google.android.apps.books
adb shell pm disable-user --user 0 com.google.android.apps.docs
adb shell pm disable-user --user 0 com.google.android.apps.magazines
adb shell pm disable-user --user 0 com.google.android.apps.maps
adb shell pm disable-user --user 0 com.google.android.apps.plus
adb shell pm disable-user --user 0 com.google.android.gm
adb shell pm disable-user --user 0 com.google.android.music
adb shell pm disable-user --user 0 com.google.android.play.games
adb shell pm disable-user --user 0 com.google.android.street
adb shell pm disable-user --user 0 com.google.android.talk
adb shell pm disable-user --user 0 com.google.android.videos
adb shell pm disable-user --user 0 com.google.android.youtube
adb shell pm disable-user --user 0 com.motorola.email
adb shell pm disable-user --user 0 com.motorola.fmplayer
adb shell pm disable-user --user 0 com.motorola.vzw.cloudsetup
adb shell pm disable-user --user 0 com.verizon.messaging.vzmsgs
adb shell pm disable-user --user 0 com.verizontelematics.verizonhum
adb shell pm disable-user --user 0 com.zte.videoplayer
adb shell pm disable-user --user 0 com.google.android.apps.docs.editors.docs
adb shell pm disable-user --user 0 com.google.android.apps.docs.editors.sheets
adb shell pm disable-user --user 0 com.google.android.apps.docs.editors.slides
adb shell pm disable-user --user 0 com.google.android.apps.messaging
adb shell pm disable-user --user 0 com.google.android.apps.photos
adb shell pm disable-user --user 0 com.google.android.apps.tachyon
adb shell pm disable-user --user 0 com.google.android.calculator
adb shell pm disable-user --user 0 com.google.android.calendar
adb shell pm enable com.google.android.googlequicksearchbox
adb shell pm disable-user --user 0 com.motorola.android.fmradio
adb shell pm disable-user --user 0 com.motorola.ccc.notification
adb shell pm disable-user --user 0 com.motorola.genie
adb shell pm disable-user --user 0 com.motorola.moto
adb shell pm disable-user --user 0 com.motorola.motocare
adb shell pm disable-user --user 0 com.motorola.motocare.internal
adb shell pm disable-user --user 0 com.motorola.mya
adb shell pm disable-user --user 0 com.motorola.mya.fmwkwrapper
adb shell pm disable-user --user 0 com.motorola.vzw.loader
adb shell pm disable-user --user 0 com.motorola.vzw.phone.extensions
adb shell pm disable-user --user 0 com.motorola.vzw.provider
adb shell pm disable-user --user 0 com.motorola.vzw.settings.extensions
adb shell pm disable-user --user 0 com.staplegames.blocksClassicSGGP
adb shell pm disable-user --user 0 com.staplegames.wordsearch
adb shell pm disable-user --user 0 com.vcast.mediamanager
adb shell pm disable-user --user 0 com.verizon.mips.services
adb shell pm disable-user --user 0 com.vzw.hss.myverizon
adb shell pm enable com.google.android.marvin.talkback
adb shell pm disable-user --user 0 com.gotv.nflgamecenter.us.lite
adb shell pm disable-user --user 0 com.vznavigator.Generic
adb shell pm disable-user --user 0 com.google.android.apps.youtube.music



