### Fix project 

* ui: unitit väärin ja muita pieniä korjauksia
* ui: mittaus ei jatku jos lataa db:stä vanhan ja alkaa mittaamaan
* ui: mittaus piste buffer laskuri ei päivity mittauksen aikana
* ui: referenssi trajektoria ei voi lataa mittauksen kanssa päällekkäin
* ui: aika-akseli on unix sekunteina, voisi olla sekunnit skriptin alusta lähtien esim
* ui: reset pitäisi resettaa myös estimaattorin tilavektori
* ui: kuinka monta pistettä näkyvillä trajektorissa tällä hetkellä
* ui: 2D graphs for x,y,z similar to acceleration
* ui: real plot (reference trajectory)
* ui: akselit ei ole samassa skaalassa

* algo: ekf driftaa
* algo: gravitaatio kompensaatio
* algo: IMU kalibrointi

* ros: estimator reset service
