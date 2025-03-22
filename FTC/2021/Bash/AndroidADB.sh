# brew install android-platform-tools

adb devices

# for f in *.apk ; do adb install "$f" ; done

# pm list packages | grep -i 'talk'

# adb install ../apk/BarcodeScanner.apk
# adb install ../apk/APKExport.apk

adb shell
pm block com.google.android.music
pm block com.nuance.swype.input
pm block com.android.browser
pm block com.google.android.youtube
pm block com.google.android.talk
pm block com.google.android.apps.plus
pm block com.google.android.apps.books
pm block com.google.android.apps.docs
pm block com.google.android.apps.magazines
pm block com.google.android.play.games
pm block com.google.android.videos
pm block com.google.android.apps.maps
pm block com.google.android.gm
pm block com.google.android.street
pm unblock com.android.chrome
pm block com.zte.videoplayer
pm block com.android.contacts
pm block com.android.email
pm block com.android.dialer
pm block com.android.mms
pm block com.android.voicedialer
pm block com.coremobility.app.vnotes
pm block cn.zte.music
pm block cn.zte.recorder
pm block com.sprint.w.installer
pm block com.qualcomm.qti.networksetting

