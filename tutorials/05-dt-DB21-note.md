# dt-DB21-note

DB21 Tutorial

```
https://docs.duckietown.org/daffy/opmanual_duckiebot/out/assembling_duckiebot_db21.html
```

| Our DB in 2018          | DB21        | Questions/Suggestions      |
| ----------------------- | ----------- | -------------------------- |
| [Laptop]Ubuntu16+VMWare(Virtual Box)| Laptop要有Ubuntu20.04, VMWare應該也可以，Nick是用VMWare, 我記得Andrea Censi之前也是用MacVMWare | | 
| [Token]之前不需要  | 必須Duckietown 網站的token, Dashboard 會用到 | 可以一人擁有多台嗎？有分個人的和可多人使用的帳號，後者可以有多個token |
| [Shell]之前沒有dts，所以很多指令會比較亂無法管理 | Duckietown Shell (dts)dts 的abstract class 定義在https://github.com/duckietown/duckietown-shell, 指令的實作在 https://github.com/duckietown/duckietown-shell-commands | 指令有點小複雜，不過是很值得學習的，能把很多docker裡面功能管理的很好，之後會在general meeting介紹 |
| [SD Card]燒錄sd card 以前是用dd燒錄後重設網路設定，重設/etc/hostname 需要自行下載正確的image檔案 | dts init_sd_card - 要先設定好網路WiFi帳密，也就是預設laptop 及 duckiebot會在同一個router無線網路- 會預設 duckiebot hostname我統一設定arg- Duckietown 好像有用Git lfs, 去下載image檔。lfs這個在2018年使用時有點問題，現在或許比較好用了，然而使用的organization 會有流量和空間的限制，會需要花錢購買，Duckietown 那邊應該是有做什麼升級，我們這邊是使用者應該就比較沒有影響. This will use the latest image. | [問題] 因為我們會使用的環境可能有622 627 759 632 要是有一樣的 router SSID and password會比較方便，看是不是既有的可以改一下，還是買幾台新的然後都要用5G連線避免干擾的問題？目前較簡單的解法是在燒錄時設定多個wifi，或是編輯/etc/wpa_supplicant.conf檔案 [問題] RPi SD Card燒錄後是否可以使用？是否需要中在小鴨車上才能做第一次開機？這個之後要給autonomy box以及anchor的RPi使用第一次開機不需要裝在小鴨車上 | 
| [First Boot Initialization]之前對拷完成就可以開機沒有檢查硬體 | 第一次開機不需要裝在小鴨車上 | 燒錄後，第一次開機initialization前會建議先用對拷機複製幾片，以避免第一次開機失敗建議確認Jetson Nano是好的（例如用Duckieboat SD卡確認可以開機）[問題] 第一次開機initialztion，究竟是完成了哪些以前要手動設定的？any ssh key? 照推斷應該都和init sd card給的參數有關 |
| [Finish Initialization]之前不用 | initialization 後用 dts fleet指令確認 | [問題] 第一次開機完成後，是否會有桌面？還是只有command line? 有桌面的話好像可以用AnyDesk?[問題] 第一次開機完成後，user 帳號密碼是什麼？ |
| [List all robots]之前沒有這樣的功能，比較是先每一台設定好固定IP後，在一台一台找 | dts fleet discovery會以hostname為主，不需要設定固定IP | [問題] 如何讓一台machine被找到，原理為何？[問題] 第一次開機完成後，萬一有兩台小鴨車用相同的duckiebot名稱 是可以的嗎？這樣dts fleet會不會有問題？要是有問題的話看起來就每一台小鴨車都要做一次燒錄 sd card的動作 |
If hostname.local is not found?

Containers: what open the compose to get dashboard?

Containers: check each one doing

dts fleet discover: how to let dts find my machine?

dts for xbee


















