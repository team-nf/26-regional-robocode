# Optimizasyon ve Iyilestirme Onerileri
**Tarih:** 2026-03-10
**Reviewer:** Claude Opus 4.6

---

## Genel Bakis

Bu dokuman, bug fix'lerden sonra yapilabilecek performans ve kod kalitesi iyilestirmelerini listeler.
Amac: karmasikligi artirmadan, gereksiz isleri azaltmak ve kodu daha okunakli hale getirmek.

---

## 1. SmartDashboard.get() Periodic Dongulerde Kullanilmamali

**Sorun:** `SmartDashboard.getBoolean()` / `getNumber()` her cagirildiginda NetworkTables uzerinden okuma yapar. Bu her 20ms'de bir gereksiz I/O demek.

**Etkilenen Dosyalar:**

| Dosya | Satir | Okunan Key |
|-------|-------|------------|
| CommandSwerveDrivetrain.java | 287-298 | `Conf/AutoAimEnabled` |
| TheMachine.java | 472-479 | `Conf/HasLotsOfFuel` |
| Localization.java | 89, 99 | `Conf/LL-Left_Enabled`, `Conf/LL-Right_Enabled` |
| Localization.java | 162, 172 | Ayni key'ler (MT1 icinde) |
| Localization.java | 253 | `Conf/DisabledLocoEnabled` |

**Cozum:** `NetworkTableEntry` kullanarak listener-based yaklasim veya degiskenleri sadece bir kere cache'leyip SmartDashboard'a `putBoolean` ile geri yazmak.

```java
// ONCE (periodic icinde her seferinde):
autoAimEnabled = SmartDashboard.getBoolean("Conf/AutoAimEnabled", autoAimEnabled);

// SONRA (bir kere baglayip cache):
private final NetworkTableEntry autoAimEntry =
    NetworkTableInstance.getDefault().getTable("SmartDashboard")
        .getEntry("Conf/AutoAimEnabled");

// Constructor'da:
autoAimEntry.setBoolean(autoAimEnabled);

// Periodic'te:
autoAimEnabled = autoAimEntry.getBoolean(autoAimEnabled);
// (Entry'ler cache'li calisir, her seferinde lookup yapmaz)
```

**Alternatif basit yol:** Degiskeni sadece DriverStation baglanti event'inde veya bir counter ile her 25 cycle'da bir oku:
```java
private int dashboardReadCounter = 0;

// periodic icinde:
if (dashboardReadCounter++ % 25 == 0) {
    autoAimEnabled = SmartDashboard.getBoolean("Conf/AutoAimEnabled", autoAimEnabled);
}
```

---

## 2. getState() Tekrarli Cagrimi

**Sorun:** `CommandSwerveDrivetrain.updateSwerveData()` icinde `getState()` 4 kere arki arkaya cagriliyor. Her cagri ayni veriyi donduruyor ama gereksiz method call overhead var.

**Dosya:** `CommandSwerveDrivetrain.java:352-365`

```java
// ONCE:
swerveData.robotPose = getState().Pose;
swerveData.robotVelocityX = MetersPerSecond.of(getState().Speeds.vxMetersPerSecond);
swerveData.robotVelocityY = MetersPerSecond.of(getState().Speeds.vyMetersPerSecond);
swerveData.robotSpeed = MetersPerSecond.of(
    Math.hypot(getState().Speeds.vxMetersPerSecond, getState().Speeds.vyMetersPerSecond));
swerveData.robotAngularVelocity = RadiansPerSecond.of(getState().Speeds.omegaRadiansPerSecond);

// SONRA:
var state = getState();
swerveData.robotPose = state.Pose;
swerveData.robotVelocityX = MetersPerSecond.of(state.Speeds.vxMetersPerSecond);
swerveData.robotVelocityY = MetersPerSecond.of(state.Speeds.vyMetersPerSecond);
swerveData.robotSpeed = MetersPerSecond.of(
    Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond));
swerveData.robotAngularVelocity = RadiansPerSecond.of(state.Speeds.omegaRadiansPerSecond);
```

Ayni dosyada `getPose()` ve `getPose().getTranslation()` de tekrarlaniyor (satir 280-284).

---

## 3. Alliance Cache'leme

**Sorun:** `DriverStation.getAlliance()` mac boyunca degismez ama 11+ yerde her execute/periodic'te tekrar cagriliyor.

**Cozum:** Bir kere cache'le, `teleopInit`/`autonomousInit`'te guncelle:

```java
// CommandSwerveDrivetrain'e ekle:
private boolean isRedAlliance = false;

public void cacheAlliance() {
    isRedAlliance = DriverStation.getAlliance()
        .map(a -> a == Alliance.Red).orElse(false);
}

// Robot.java teleopInit/autonomousInit icinden cagir:
m_robotContainer.cacheAlliance();

// Sonra her yerde:
if (isRedAlliance) { ... }  // Optional islemine gerek yok
```

**Not:** `setStartPoseInitial()` ve `SwerveAimToHub.initialize()` gibi yerlerde hala safe Optional kullanilmali cunku bunlar DS baglanmadan once calisabilir.

---

## 4. CommandScheduler.getInstance() Tekrari

**Sorun:** State machine'lerde `CommandScheduler.getInstance()` her case icinde 2 kere cagriliyor (isScheduled + schedule). Tum subsystem'lerde 35+ tekrar var.

**Dosyalar:** TheMachine.java, ShooterSubsystem.java, HopperSubsystem.java, FeederSubsystem.java, IntakeSubsystem.java

```java
// ONCE:
case ZERO:
    if(!CommandScheduler.getInstance().isScheduled(theMachineZeroAction))
    {
        CommandScheduler.getInstance().schedule(theMachineZeroAction);
    }
    break;

// SONRA:
public void stateMachine() {
    var scheduler = CommandScheduler.getInstance();
    switch (theMachineData.theMachineControlState) {
        case ZERO:
            if (!scheduler.isScheduled(theMachineZeroAction))
                scheduler.schedule(theMachineZeroAction);
            break;
        // ...
    }
}
```

**Daha temiz alternatif:** Ortak bir helper method:
```java
private void scheduleIfNotRunning(Command action) {
    var scheduler = CommandScheduler.getInstance();
    if (!scheduler.isScheduled(action)) {
        scheduler.schedule(action);
    }
}

// Kullanimi:
case ZERO:  scheduleIfNotRunning(theMachineZeroAction); break;
case IDLE:  scheduleIfNotRunning(theMachineIdleAction); break;
```

Bu, stateMachine() metodunu 70 satirdan 15 satira dusurur.

---

## 5. Kullanilmayan Obje Olusturma

**Sorun:** `updateSwerveData()` icinde `roundedRobotPose` olusturuluyor ama hic kullanilmiyor (satir 354-357, kullanildigi satir 359 comment'li).

**Dosya:** `CommandSwerveDrivetrain.java:354-359`

```java
// Bu 4 satiri tamamen sil:
Pose2d roundedRobotPose = new Pose2d(
    TelemetryConstants.roundTelemetry(swerveData.robotPose.getX()),
    TelemetryConstants.roundTelemetry(swerveData.robotPose.getY()),
    new Rotation2d(TelemetryConstants.roundTelemetry(swerveData.robotPose.getRotation().getRadians())));
//swerveData.field.setRobotPose(roundedRobotPose);
```

---

## 6. VecBuilder.fill() Cache'leme (Localization)

**Sorun:** `VecBuilder.fill(.6,.6,9999999)` her vision update'de yeni obje olusturuyor. Ayni degerler 4 kez tekrarlaniyor.

**Dosya:** `Localization.java:91, 101, 164, 174`

```java
// Class seviyesinde tanimla:
private static final Matrix<N3, N1> MT2_STD_DEVS = VecBuilder.fill(0.6, 0.6, 9999999);
private static final Matrix<N3, N1> MT1_STD_DEVS = VecBuilder.fill(0.5, 0.5, 9999999);

// Kullanimi:
drivetrain.setVisionMeasurementStdDevs(MT2_STD_DEVS);
```

---

## 7. Obje Allocation'lari execute() Icinden Cikarma

**Sorun:** `SwerveAimToHub` ve `SwerveAimToPass`'in execute() metodlarinda her 20ms'de `new Transform2d` + `new Rotation2d` olusturuluyor.

**Dosyalar:** `SwerveAimToHub.java:83`, `SwerveAimToPass.java:90`

```java
// ONCE (execute icinde, 50 Hz):
swerveDrivetrain.updateSwerveErrors(
    robotPose.plus(new Transform2d(0.0, 0.0, new Rotation2d(averageError))));

// SONRA (field olarak tanimla, execute'da yeniden kullan):
private Transform2d errorTransform = new Transform2d();

// execute icinde:
errorTransform = new Transform2d(0.0, 0.0, Rotation2d.fromRadians(averageError));
swerveDrivetrain.updateSwerveErrors(robotPose.plus(errorTransform));
```

**Not:** Java GC bunlari kolayca handle eder, ama FRC'de deterministik loop timing onemli oldugu icin azaltmak iyi pratik.

---

## 8. Yorum Satiri Temizligi

Birden fazla dosyada artik kullanilmayan, comment'lenmis kod bloklari var. Bunlar git history'de zaten mevcut, kodda tutmanin anlami yok.

**Temizlenecek Yerler:**
- `CommandSwerveDrivetrain.java:273-276` - Commented-out field telemetry
- `CommandSwerveDrivetrain.java:302-312` - Commented-out neutral mode switching
- `ShooterCalculator.java:58-67, 83-92, 107-113` - Commented-out SmartDashboard manual override
- `ShooterCalculator.java:138-146` - `hoodAngleFormulaOLD` eski formula

---

## 9. SwerveGetIntoShootAreaCommand Hub Pose Tekrari

**Sorun:** `execute()` icinde her seferinde `new Pose2d(4.61, 4.1, ...)` ve `new Pose2d(11.92, 4.1, ...)` olusturuluyor.

**Dosya:** `SwerveGetIntoShootAreaCommand.java:48-52`

```java
// ONCE (execute icinde):
hubAimPose = new Pose2d(4.61, 4.1, new Rotation2d());
// else
hubAimPose = new Pose2d(11.92, 4.1, new Rotation2d());

// SONRA (initialize icinde bir kere):
@Override
public void initialize() {
    boolean isRed = DriverStation.getAlliance()
        .map(a -> a == Alliance.Red).orElse(false);
    hubAimPose = isRed
        ? PoseConstants.RED_HUB_AIM_POSE
        : PoseConstants.BLUE_HUB_AIM_POSE;
}
```

Bu hem allocation'i azaltir hem magic number'lari kaldirir.

---

## 10. Telemetry Gate Eksikligi

**Sorun:** Bazi telemetry cagrilari `TelemetryConstants` kontrolu olmadan her cycle'da calisiyor.

**Dosyalar:**
- `IntakeSubsystem.java:288-289` - `putBoolean` TelemetryConstants gate'siz
- `CommandSwerveDrivetrain.java:342-345` - `putData`, `putBoolean` her cycle

```java
// Mevcut pattern (diger yerlerde kullanilan):
if (TelemetryConstants.SHOULD_SHOOTER_COMMUNICATE) {
    SmartDashboard.putNumber("...", value);
}

// Eksik olan yerlere de ayni pattern ekle:
if (TelemetryConstants.SHOULD_SWERVE_COMMUNICATE) {
    confField2d.setRobotPose(...);
    SmartDashboard.putData("Conf/Field", confField2d);
}
```

---

## Oncelik Sirasi

### Kolay ve Etkili (Hemen yapilabilir):
1. **#2** - getState() cache (1 satir degisiklik, net iyilesme)
2. **#5** - roundedRobotPose silme (4 satir sil)
3. **#4** - scheduleIfNotRunning helper (tum subsystem'leri sadeletirir)
4. **#6** - VecBuilder cache (4 satir -> 2 static field)

### Orta Efor:
5. **#3** - Alliance cache (birden fazla dosyada degisiklik)
6. **#1** - SmartDashboard.get throttling (basit counter ile)
7. **#7** - execute() obje allocation azaltma
8. **#9** - Hub pose magic number'lari constant'a tasima

### Temizlik (Zaman buldukca):
9. **#8** - Comment temizligi
10. **#10** - Telemetry gate eksikleri

---

## Yapilmamasi Gereken Seyler

- **Subsystem mimarisini degistirmek** - Mevcut state machine pattern iyi calisiyor
- **Command yapisi degistirmek** - WPILib command-based zaten iyi organize
- **Over-engineering** - NetworkTable listener pattern'i eklemek cok komplex, basit cache/throttle yeter
- **Premature optimization** - 20ms loop icinde 2-3 extra method call performans sorunu yaratmaz, ama temiz kod icin duzeltmek iyi
