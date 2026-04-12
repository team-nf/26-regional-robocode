# Codex Review - 2026-03-09

## Overview

- Inceleme tipi: statik kaynak kod incelemesi. Kullanici istegiyle derleme ve test yapilmadi.
- Odak alanlari: `RobotContainer`, `CommandSwerveDrivetrain`, `TheMachine`, `Localization`, auto/path akislari.
- Toplam bulgu: 7 adet.
- Dagilim: 2 kritik, 4 yuksek, 1 orta.
- Ana tema: alliance bilgisinin guvensiz kullanimi ve state-machine akislarinda davranissal kopmalar.

## Bulgular

### 1. [Kritik] `DriverStation.getAlliance().get()` kullanimi robotu acilis ve calisma sirasinda dusurebilir

- Referanslar: `src/main/java/frc/robot/Subsytems/Swerve/CommandSwerveDrivetrain.java:278`, `src/main/java/frc/robot/Subsytems/Swerve/CommandSwerveDrivetrain.java:636`, `src/main/java/frc/robot/Subsytems/Swerve/Commands/SwerveTeleopCommand.java:54`, `src/main/java/frc/robot/Subsytems/Swerve/Commands/SwerveAimToHub.java:50`, `src/main/java/frc/robot/Subsytems/Swerve/Commands/SwerveAimToPass.java:75`
- Neden oluyor:
  `DriverStation.getAlliance()` bir `Optional` donduruyor. Kod ise bircok yerde bunu dogrudan `.get()` ile aciyor. Driver Station henuz tam baglanmadiysa, sim ortaminda alliance bilinmiyorsa veya match metadata gec gelirse bu `Optional` bos gelir.
- Neye sebep olur:
  `CommandSwerveDrivetrain` constructor'i icindeki `setStartPoseInitial()` cagrisi daha robot acilisinda `NoSuchElementException` atabilir. Daha sonrasinda default teleop komutu, aim komutlari ve path/pose secimi de ayni nedenle runtime'da robot kodunu dusurebilir.
- Nasil duzeltilir:
  Tek bir yardimci metod uretip alliance bilgisini guvenli sekilde okuyun. Ornek yaklasim:
  `DriverStation.getAlliance().orElse(Alliance.Blue)` ya da `ifPresent/else` ile fallback davranisi tanimlayin.
  Ozellikle constructor ve periodic akislarda `.get()` tamamen kaldirilmali.

### 2. [Kritik] `Localization.addVisionMeasurementMT1()` sag kamera filtresinde yanlis degiskeni kullaniyor ve NPE uretebiliyor

- Referanslar: `src/main/java/frc/robot/Utils/Localization.java:146-155`
- Neden oluyor:
  Sag Limelight blogunda `limelightMeasurementRight` kontrol edildikten sonra mesafe filtresi `limelightMeasurementLeft.avgTagDist` uzerinden yapiliyor. Ustelik reject flag olarak da `doRejectRight` yerine `doRejectLeft` set ediliyor.
- Neye sebep olur:
  Sol kamera `null`, sag kamera doluysa kod `limelightMeasurementLeft.avgTagDist` satirinda `NullPointerException` atabilir. Sol kamera dolu olsa bile sag kameranin uzaklik filtresi hic dogru uygulanmaz; kotu sag vision olcumleri sisteme kabul edilebilir.
- Nasil duzeltilir:
  Sag kamera blogunda hem kontrolu hem reject flag'i saga cevirin:
  `if (limelightMeasurementRight.avgTagDist > 3.5) { doRejectRight = true; }`
  Bu blok icin sol ve sag kamerayi ayri methodlara ayirmak da benzer copy/paste hatalarini azaltir.

### 3. [Yuksek] `GET_READY` aksiyonu hazirlik yapmak yerine tum makine state'ini dogrudan `SHOOT` durumuna geciriyor

- Referanslar: `src/main/java/frc/robot/Subsytems/TheMachine/StateActions/TheMachineGetReadyAction.java:11-16`, `src/main/java/frc/robot/Subsytems/TheMachine/TheMachine.java:294-303`
- Neden oluyor:
  `TheMachineGetReadyAction` icinde `theMachine.shootRequest()` cagriliyor. Bu, shooter subsystem'ini hazirlamak yerine tum `TheMachine` state'ini `SHOOT` olarak degistiren request.
- Neye sebep olur:
  `GET_READY` state'i kararsiz hale gelir ve bir scheduler dongusu icinde `SHOOT` state'ine atlanir. Bu da aim bitmeden veya istenen senkronizasyon kurulmadan feed/shoot akislarinin erken baslamasina neden olabilir.
- Nasil duzeltilir:
  Burada tum makine state request'i degil, alt sistem request'i kullanilmali:
  `theMachine.shooterShootRequest()`
  Boylece `TheMachine` state'i `GET_READY` olarak kalir, sadece shooter hedef hiz/aciya gider.

### 4. [Yuksek] `GET_READY_PAS` aksiyonu iki ayri mantik hatasi tasiyor

- Referanslar: `src/main/java/frc/robot/Subsytems/TheMachine/StateActions/TheMachineGetReadyActionPas.java:11-16`, `src/main/java/frc/robot/Subsytems/TheMachine/TheMachine.java:302-311`
- Neden oluyor:
  Birinci hata, `theMachine.testRequest()` ile yine tum makine state'i `TEST` durumuna cekiliyor.
  Ikinci hata, komutun bitis kosulu `GET_READY_PAS` yerine `GET_READY` ile kontrol ediliyor.
- Neye sebep olur:
  Pass hazirlik akisi scheduler'a girer girmez ya state degistirir ya da `until` kosulu nedeniyle aninda biter. Sonuc olarak `GET_READY_PAS` modu pratikte stabil calismaz.
- Nasil duzeltilir:
  `theMachine.testRequest()` yerine `theMachine.shooterTestRequest()` kullanin.
  `until(() -> theMachine.getState() != TheMachineControlState.GET_READY_PAS)` olarak duzeltin.
  Bu iki duzeltme birlikte yapilmali; biri tek basina akisi kurtarmaz.

### 5. [Yuksek] Shoot komut zincirlerinde `andThen(...)` pratikte hic calismiyor

- Referanslar: `src/main/java/frc/robot/RobotContainer.java:129-130`, `src/main/java/frc/robot/RobotContainer.java:149-150`, `src/main/java/frc/robot/RobotContainer.java:155-156`, `src/main/java/frc/robot/RobotContainer.java:172-176`, `src/main/java/frc/robot/Subsytems/Swerve/Commands/SwerveAimToHub.java:117-119`, `src/main/java/frc/robot/Subsytems/Swerve/Commands/SwerveAimToPass.java:118-120`
- Neden oluyor:
  `aimToHub()` ve `aimToPass()` komutlari `isFinished()` icinde her zaman `false` donuyor. Bunlar `alongWith(...).andThen(...)` zincirinde kullanildigi icin paralel grubun tamamlanmasi bekleniyor, ancak aim komutu hic bitmediginden `andThen(m_theMachine.shootRequest())` veya `andThen(m_theMachine.testRequest())` satirlarina ulasilamiyor.
- Neye sebep olur:
  Sag bumper, sol bumper ve `AimAndShoot` named command'i beklenen sekilde "once aim ol, sonra ates et" davranisini vermiyor. `AimAndShoot` tarafinda komut buyuk ihtimalle timeout'a dusup ates etmeden sonraki state'e geciyor.
- Nasil duzeltilir:
  Iki temiz yol var:
  `aimToHub().until(m_swerveDrivetrain::isAimed)` benzeri sonlanan bir aim komutu uretmek.
  Ya da `deadlineWith` / `raceWith` kullanip bitis kosulunu `waitForAtAim()` uzerinden kurmak.
  Mevcut `alongWith(...).andThen(...)` yapisi, aim komutu sonsuz kaldigi surece dogru davranmaz.

### 6. [Yuksek] `start` tusu ayni anda birden fazla cakisan is yapiyor; simde NPE, gercekte pose reset yarisi var

- Referanslar: `src/main/java/frc/robot/RobotContainer.java:162-166`, `src/main/java/frc/robot/RobotContainer.java:244`, `src/main/java/frc/robot/Subsytems/Swerve/CommandSwerveDrivetrain.java:628`, `src/main/java/frc/robot/Subsytems/Swerve/CommandSwerveDrivetrain.java:798-803`
- Neden oluyor:
  `start()` trigger'ina iki farkli pose reset komutu baglanmis. Simulasyonda buna ek olarak `matchTracker.startMatchCommand()` da baglaniyor. Fakat `m_localization` sadece `Robot.isReal()` iken initialize ediliyor.
- Neye sebep olur:
  Simulasyonda `start` tusuna basmak `m_localization.resetWithMT1()` uzerinden `NullPointerException` uretebilir.
  Gercek robotta ise ayni event'te hem start pose reset'i hem MT1 reset'i calisip hangi pose'un son kazanacagi belirsiz hale gelir.
- Nasil duzeltilir:
  `start` tusu icin tek bir komut akisi tanimlayin.
  `resetPoseWithMT1Command()` icinde `m_localization == null` guard'i ekleyin.
  Eger iki reset davranisi da lazimsa bunlari farkli butonlara ayirin veya acik bir oncelik ile `andThen` zincirine koyun.

### 7. [Orta] Path dosyasi yukleme hatalari loglanip `null` ile devam ediliyor

- Referanslar: `src/main/java/frc/robot/Subsytems/Swerve/CommandSwerveDrivetrain.java:569-618`, `src/main/java/frc/robot/Subsytems/Swerve/CommandSwerveDrivetrain.java:667-692`, `src/main/java/frc/robot/Subsytems/Swerve/CommandSwerveDrivetrain.java:835-883`
- Neden oluyor:
  `PathPlannerPath.fromPathFile(...)` exception firlatirsa kod sadece stack trace basiyor ve ardindan `pathfindToPath(path)`, `followPath(path)` veya `pathfindThenFollowPath(path)` cagrilarina `null` geciyor.
- Neye sebep olur:
  Eksik/bozuk path dosyasinda hata erken ve anlamli bir sekilde raporlanmak yerine daha sonra `NullPointerException` ile patlar. Ustelik bu komutlar `RobotContainer` icinde `NamedCommands.registerCommand(...)` sirasinda olusturuldugu icin hata robot init asamasina tasinabilir.
- Nasil duzeltilir:
  Path yuklenemezse `Commands.none()`, `Commands.print(...)` veya bilincli bir `IllegalStateException` dondurun.
  `path == null` icin guard koymadan mevcut akis devam etmemeli.

## Kisa Sonuc

- En kritik iki konu: alliance bilgisinin guvensiz okunmasi ve `Localization` icindeki sag kamera bug'i.
- Davranissal olarak en pahali konu: `GET_READY` / `GET_READY_PAS` ve shoot zincirlerinin birbirini bozmasi.
- Bu repo icin ilk duzeltme paketi bence su sirayla olmali: alliance guard'lari, `Localization` kopyala-yapistir bug'i, sonra `TheMachine` hazirlik ve shoot akislari.
