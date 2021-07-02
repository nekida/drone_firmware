/*
Файл target.h специфичен для каждой модели
*/

int main(void)
{
    init();
    loopbackInit();

    while (true) {
        scheduler();
        processLoopback();
    }
}

/*-------------------------------------------------------------*/
void init(void)
{
// если определен макрос использования файловой системы и использования микросхемы 
// внешней памяти
#if defined(USE_FLASHFS) && defined(USE_FLASH_M25P16)
    bool flashDeviceInitialized = false;
#endif

//USE_HAL_DRIVER нигде не определен. М. б. есть в cmake или make файлах
#ifdef USE_HAL_DRIVER
    HAL_Init();
#endif

    systemState = SYSTEM_STATE_INITIALISING;	// задаем состояние
    printfSupportInit();						// инициализируем консоль

    // Initialize system and CPU clocks to their initial values
    /*
    Настройка ФАПЧ
    Установка адреса таблицы прерываний
    Выключение тактирования OTG_FS
    enableGPIOPowerUsageAndNoiseReductions() - Включение периферии:
    	-Порты
		-CRC
		-Интерфейс флэш памяти
		-Банки ОЗУ
		-BKPSRAM
		-DMA
		-Таймеры
		-WWDG
		-SPI
		-USART
		-I2C
		-PWR
		-ADC
		-SYSCFG
	cycleCounterInit() - инит микросекундного счетчика отладки DWT
	Инит SysTick
    */
    systemInit();

    __enable_irq();

    // initialize IO (needed for all IO operations)
    IOInitGlobal(); //инициализация пинов

#ifdef USE_HARDWARE_REVISION_DETECTION // определено в target.h
    detectHardwareRevision();	//Проверка ревизии по уровню на ножках
	//Инверсия ножки uart1
/*
         TODO: после того, как группы параметров установлены, выходы инвертора
         можно превратить в простые выходы ввода-вывода и просто настроить их
         HI или LO в конфигурации
 */
#endif

#ifdef USE_BRUSHED_ESC_AUTODETECT
    detectBrushedESC();	//определение движков с регулятором оборотов (ESC)
#endif

    initEEPROM();
    ensureEEPROMContainsValidData();
    readEEPROM();

#ifdef USE_UNDERCLOCK //определяется в common.h в зависимости от серии МК (определяется при STM32F3)
    // Re-initialize system clock to their final values (if necessary)
    systemClockSetup(systemConfig()->cpuUnderclock);
#else
    systemClockSetup(false);
#endif

#ifdef USE_I2C 		//определенно в target.h
    i2cSetSpeed(systemConfig()->i2c_speed); //задание 2-м модулям I2C скорости
#endif

#ifdef USE_HARDWARE_PREBOOT_SETUP	//определенно в target.h
    initialisePreBootHardware();
#endif

    systemState |= SYSTEM_STATE_CONFIG_LOADED;

    debugMode = systemConfig()->debug_mode;

    // Latch active features to be used for feature() in the remainder of init().
    latchActiveFeatures();	//последние фичи

    ledInit(false);

#ifdef USE_EXTI	//определенно в target.h
    EXTIInit(); //тактирование EXTI
#endif

#ifdef USE_SPEKTRUM_BIND	//определенно в target.h
    if (rxConfig()->receiverType == RX_TYPE_SERIAL) {
        switch (rxConfig()->serialrx_provider) {    // инициализация протоколов передачи 
            case SERIALRX_SPEKTRUM1024:
            case SERIALRX_SPEKTRUM2048:
                // Spektrum satellite binding if enabled on startup.
                // Must be called before that 100ms sleep so that we don't lose satellite's binding window after startup.
                // The rest of Spektrum initialization will happen later - via spektrumInit()
                spektrumBind(rxConfigMutable());
                break;
        }
    }
#endif

#ifdef USE_VCP	//определенно в target.h
    // Early initialize USB hardware
    usbVcpInitHardware();
#endif

    timerInit();  // timer must be initialized before any channel is allocated

#if defined(AVOID_UART2_FOR_PWM_PPM)
    serialInit(feature(FEATURE_SOFTSERIAL),
            (rxConfig()->receiverType == RX_TYPE_PPM) ? SERIAL_PORT_USART2 : SERIAL_PORT_NONE);
#elif defined(AVOID_UART3_FOR_PWM_PPM)
    serialInit(feature(FEATURE_SOFTSERIAL),
            (rxConfig()->receiverType == RX_TYPE_PPM) ? SERIAL_PORT_USART3 : SERIAL_PORT_NONE);
#else
    serialInit(feature(FEATURE_SOFTSERIAL), SERIAL_PORT_NONE);
#endif

    // Initialize MSP serial ports here so LOG can share a port with MSP.
    // XXX: Don't call mspFcInit() yet, since it initializes the boxes and needs
    // to run after the sensors have been detected.
    mspSerialInit();    // инициализация последовательных портов

#if defined(USE_DJI_HD_OSD) // определено в common.h
    // DJI OSD uses a special flavour of MSP (subset of Betaflight 4.1.1 MSP) - process as part of serial task
    djiOsdSerialInit(); // инициализация последовательных портов
#endif

#if defined(USE_SMARTPORT_MASTER)   // определено в common.h
    smartportMasterInit();  // инициализация последовательных портов
#endif

#if defined(USE_LOG)
    // LOG might use serial output, so we only can init it after serial port is ready
    // From this point on we can use LOG_*() to produce real-time debugging information
    logInit();  // инициализация последовательного порта для печати логированных данных
#endif

#ifdef USE_PROGRAMMING_FRAMEWORK    // определено в common.h
    gvInit();   // инициализация массива с глобальными переменными
#endif

    // Initialize servo and motor mixers
    // This needs to be called early to set up platform type correctly and count required motors & servos
    /*
    // Инициализация сервомашин и моторных смесителей
     // Это необходимо вызвать заранее, чтобы правильно настроить тип платформы и подсчитать требуемые двигатели и сервоприводы
    */
    servosInit();
    mixerUpdateStateFlags();    //инициализация в зависимости от модели
    mixerInit();                //инициализация физических параметров двигателей

    // Some sanity checking
    if (motorConfig()->motorPwmProtocol == PWM_TYPE_BRUSHED) {
        featureClear(FEATURE_REVERSIBLE_MOTORS);
    }
    if (!STATE(ALTITUDE_CONTROL)) {
        featureClear(FEATURE_AIRMODE);
    }

    // Initialize motor and servo outpus
    (pwmMotorAndServoInit()) ? (DISABLE_ARMING_FLAG(ARMING_DISABLED_PWM_OUTPUT_ERROR)) : (ENABLE_ARMING_FLAG(ARMING_DISABLED_PWM_OUTPUT_ERROR));    //выставление или сброс флага 

    systemState |= SYSTEM_STATE_MOTORS_READY;

#ifdef USE_ESC_SENSOR // определен в target.h
    // DSHOT supports a dedicated wire ESC telemetry. Kick off the ESC-sensor receiver initialization
    // We may, however, do listen_only, so need to init this anyway
    // Initialize escSensor after having done it with outputs
    /*
    // DSHOT поддерживает телеметрию ESC по выделенному кабелю. Начать инициализацию приемника ESC-сенсора
     // Тем не менее, мы можем использовать listen_only, поэтому все равно нужно это инициализировать
     // Инициализируем escSensor после того, как сделали это с выходами
    */
    escSensorInitialize();  // инициализация последовательного порта для ESC датчиков
#endif

#ifdef BEEPER   // определен в target.h  на определенную ножку
    beeperDevConfig_t beeperDevConfig = {
        .ioTag = IO_TAG(BEEPER),
        /*
        IO_TAG(x) -> DEFIO_TAG(pinid) -> CONCAT(DEFIO_TAG__, x) -> CONCAT_HELPER(x, y) -> CONCAT_HELPER(x,y) x ## y
        т.е., тут при BEEPER = PB6 будет DEFIO_TAG__PB6

        #if DEFIO_PORT_B_USED_MASK & BIT(6)
            #define DEFIO_TAG__PB6 DEFIO_TAG_MAKE(DEFIO_GPIOID__B, 6)
        #else
            # define DEFIO_TAG__PB6 defio_error_PB6_is_not_supported_on_TARGET

        #define DEFIO_TAG_MAKE(gpioid, pin) ((((gpioid) + 1) << 4) | (pin))
        DEFIO_TAG_MAKE(DEFIO_GPIOID__B, 6) -> ((((DEFIO_GPIOID__B) + 1) << 4) | (6)) -> ((((1) + 1) << 4) | 6) -> 0b00100110
        */
#ifdef BEEPER_INVERTED
        .isOD = false,
        .isInverted = true
#else
        .isOD = true,
        .isInverted = false
#endif
    };

    beeperInit(&beeperDevConfig);   // инициализация beeper, в том числе ножки, частоты шим
#endif

#ifdef USE_LIGHTS   // хз где определено
    lightsInit();   // инициализация ноги с диодом
#endif

#ifdef USE_UART_INVERTER    // определен в target.h
    uartInverterInit();     // инициализация ножек uart в зависимости от макроса INVERTER_PIN_UARTx_xX
#endif

    // Initialize buses
    /*
    extern const busDeviceDescriptor_t __busdev_registry_start[];
    extern const busDeviceDescriptor_t __busdev_registry_end[];
    #define BUSDEV_REGISTER_ATTRIBUTES __attribute__ ((section(".busdev_registry"), used, aligned(4)))

    в линкер скрипте
      .busdev_registry :
      {
        PROVIDE_HIDDEN (__busdev_registry_start = .);
        KEEP (*(.busdev_registry))                      //сохранить раздел
        KEEP (*(SORT(.busdev_registry.*)))              //сохранить раздел (сортирует файлы или разделы в порядке возрастания по имени перед помещением их в выходной файл)
        PROVIDE_HIDDEN (__busdev_registry_end = .);
      } >FLASH
      Сделано для возможности сборки на эпле
    */
    busInit();

#ifdef USE_HARDWARE_REVISION_DETECTION
    updateHardwareRevision(); // проверка ревизии. в определенной ревизии освободить ножку beeper
#endif

#if defined(USE_RANGEFINDER_HCSR04) && defined(USE_SOFTSERIAL1) // USE_RANGEFINDER_HCSR04 определен в target.h && USE_SOFTSERIAL1 определен в target.h
#if defined(FURYF3) || defined(OMNIBUS) || defined(SPRACINGF3MINI)  // ничего не определено
    if ((rangefinderConfig()->rangefinder_hardware == RANGEFINDER_HCSR04) && feature(FEATURE_SOFTSERIAL)) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL1);
    }
#endif
#endif

#if defined(USE_RANGEFINDER_HCSR04) && defined(USE_SOFTSERIAL2) && defined(SPRACINGF3) // USE_RANGEFINDER_HCSR04 определен в target.h && USE_SOFTSERIAL2 определен в target.h
    if ((rangefinderConfig()->rangefinder_hardware == RANGEFINDER_HCSR04) && feature(FEATURE_SOFTSERIAL)) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL2);
    }
#endif

#ifdef USE_USB_MSC  // не определено
    /* MSC mode will start after init, but will not allow scheduler to run,
     * so there is no bottleneck in reading and writing data
     */
    /*
    / * Режим MSC запустится после инициализации, но не позволит запустить планировщик,
      * поэтому нет узких мест при чтении и записи данных
    */
    mscInit();
#if defined(USE_FLASHFS)    // определен в target.h
        // If the blackbox device is onboard flash, then initialize and scan
        // it to identify the log files *before* starting the USB device to
        // prevent timeouts of the mass storage device.

    /*
        // Если устройство черного ящика - это встроенная флеш-память, то инициализируем и просканируем
        // это для идентификации файлов журнала * перед * запуском USB-устройства в
        // предотвращаем тайм-ауты запоминающего устройства.
    */
        if (blackboxConfig()->device == BLACKBOX_DEVICE_FLASH) {
#ifdef USE_FLASH_M25P16             // определен в target.h, внешняя микросхема флэш памяти
            // Must initialise the device to read _anything_
            /*m25p16_init(0);*/
            if (!flashDeviceInitialized) { //false, если USE_FLASHFS && USE_FLASH_M25P16
                flashDeviceInitialized = flashInit();
            }
#endif
            emfat_init_files(); //создание файла
        }
#endif

    if (mscCheckBoot() || mscCheckButton()) { //если удалось прочитать объект или кнопка нажата
        if (mscStart() == 0) {  //инициализация USB
             mscWaitForButton(); //макрос использования кнопки не определен 
        } else {
             NVIC_SystemReset();    //софтовый сброс
        }
    }
#endif

#ifdef USE_I2C  // определен в target.h
#ifdef USE_I2C_DEVICE_1 // определен в target.h
    #ifdef I2C_DEVICE_1_SHARES_UART3    //нигде не определено
        if (!doesConfigurationUsePort(SERIAL_PORT_USART3)) {
            i2cInit(I2CDEV_1);
        }
    #else
            i2cInit(I2CDEV_1);  // инит и2ц: пины, тактирование, скорость, адрес
    #endif
#endif

#ifdef USE_I2C_DEVICE_2 // определен в target.h
    #ifdef I2C_DEVICE_2_SHARES_UART3    // определен в target.h
        if (!doesConfigurationUsePort(SERIAL_PORT_USART3)) { // если не удалось найти свободный последоваетльный порт
            i2cInit(I2CDEV_2);
        }
    #else
            i2cInit(I2CDEV_2);
    #endif
#endif

#ifdef USE_I2C_DEVICE_3
    i2cInit(I2CDEV_3);
#endif

#ifdef USE_I2C_DEVICE_4
    i2cInit(I2CDEV_4);
#endif

#ifdef USE_I2C_DEVICE_EMULATED
    #ifdef I2C_DEVICE_EMULATED_SHARES_UART3
        if (!doesConfigurationUsePort(SERIAL_PORT_USART3)) {
            i2cInit(I2CDEV_EMULATED);
        }
    #else
            i2cInit(I2CDEV_EMULATED);
    #endif
#endif
#endif

#ifdef USE_ADC // определен в target.h
    drv_adc_config_t adc_params;    //объявление переменной 
    memset(&adc_params, 0, sizeof(adc_params)); //зануление переменной

    // Allocate and initialize ADC channels if features are configured - can't rely on sensor detection here, it's done later
    // Выделяем и инициализируем каналы АЦП, если функции настроены - здесь нельзя полагаться на обнаружение датчика, это будет сделано позже
    if (feature(FEATURE_VBAT)) {    // если особенность есть
        //присвоение канала 
        adc_params.adcFunctionChannel[ADC_BATTERY] = adcChannelConfig()->adcFunctionChannel[ADC_BATTERY];
    }

    if (feature(FEATURE_RSSI_ADC)) {
        adc_params.adcFunctionChannel[ADC_RSSI] = adcChannelConfig()->adcFunctionChannel[ADC_RSSI];
    }

    if (feature(FEATURE_CURRENT_METER) && batteryMetersConfig()->current.type == CURRENT_SENSOR_ADC) {
        //присвоение канала 
        adc_params.adcFunctionChannel[ADC_CURRENT] =  adcChannelConfig()->adcFunctionChannel[ADC_CURRENT];
    }

#if defined(USE_PITOT) && defined(USE_ADC) && defined(USE_PITOT_ADC)    //нет в таргете
    if (pitotmeterConfig()->pitot_hardware == PITOT_ADC || pitotmeterConfig()->pitot_hardware == PITOT_AUTODETECT) {
        adc_params.adcFunctionChannel[ADC_AIRSPEED] = adcChannelConfig()->adcFunctionChannel[ADC_AIRSPEED];
    }
#endif

    adcInit(&adc_params); //инит ацп: канал на ножку
#endif

#ifdef USE_PINIO //нет в таргете
    pinioInit();
#endif

#ifdef USE_PINIOBOX //нет в таргете
    pinioBoxInit();
#endif

#if defined(USE_GPS) || defined(USE_MAG)    //USE_GPS - в common.h, USE_MAG - в target.h
    delay(500);

    /* Extra 500ms delay prior to initialising hardware if board is cold-booting */
    /* Дополнительная задержка в 500 мс перед инициализацией оборудования, если плата выполняет холодную загрузку */
    if (!isMPUSoftReset()) {    //если не было программного сброса
        //Программный сброс возникает когда не выбрано ни sd карта ни файловая система
        //помигать диодами
        LED1_ON;
        LED0_OFF;

        for (int i = 0; i < 5; i++) {
            LED1_TOGGLE;
            LED0_TOGGLE;
            delay(100);
        }

        LED0_OFF;
        LED1_OFF;
    }
#endif

    initBoardAlignment(); //инит выравнивания платы
    /*
    если значения крена, высоты и рыскания по нулям
        выравнивание произведено
    иначе {
        инит углы поворота значения в радианах
        Построение rMat из углов Тейта – Брайана (условные обозначения X1, Y2, Z3)
        матрица вращения по углам
    }
    */

#ifdef USE_CMS //не нашей платформе
    cmsInit();
#endif

#ifdef USE_DASHBOARD //не нашей платформе
    if (feature(FEATURE_DASHBOARD)) {
        dashboardInit();
    }
#endif

#ifdef USE_GPS  //USE_GPS - в common.h
    if (feature(FEATURE_GPS)) { //фича не выставлена
        gpsPreInit();
    }
#endif

    // 1-Wire IF chip
#ifdef USE_1WIRE    //- в common.h
    owInit();   //инит ван вайр
#endif

    if (!sensorsAutodetect()) { // если не находим гироскоп на протяжении какого-либо времени
        // if gyro was not detected due to whatever reason, we give up now.
        failureMode(FAILURE_MISSING_ACC);   //минагаем диодом соответствующее количество раз и идем в ребут
    }

    systemState |= SYSTEM_STATE_SENSORS_READY;

    flashLedsAndBeep(); //зажечь диоды и бипнуть

    pidInitFilters();   //инициализация ПИД фильтров

    imuInit();  //инициализация инерциальных измерительных модулей

    // Sensors have now been detected, mspFcInit() can now be called
    // to set the boxes up
    /*
    // Датчики обнаружены, теперь можно вызвать mspFcInit ()
     // для установки боксов
    */
    mspFcInit();    //msp - это протокол "запрос - ответ"

    cliInit(serialConfig());    //инициализация командной строки

    failsafeInit(); //инит отказоустойчивости

    rxInit();   //инициализация приемника
    /*
    инит радиоканала
    инит протоколов
    */

#if defined(USE_OSD)    //- в target.h
    displayPort_t *osdDisplayPort = NULL;
#endif

#ifdef USE_OSD
    if (feature(FEATURE_OSD)) { //- в target.h
#if defined(USE_FRSKYOSD)   //- в common.h
        if (!osdDisplayPort) {  // Frsky - TX протокол 
            osdDisplayPort = frskyOSDDisplayPortInit(osdConfig()->video_system);    //инит наэкранного меню
        }
#endif
#if defined(USE_MAX7456)    //- в target.h
        // If there is a max7456 chip for the OSD and we have no
        // external OSD initialized, use it.
        /*
        // Если есть микросхема max7456 для OSD и у нас нет инициализированного внешнего OSD, 
        используйте ее.
        */
        if (!osdDisplayPort) {
            osdDisplayPort = max7456DisplayPortInit(osdConfig()->video_system);
        }
#elif defined(USE_OSD_OVER_MSP_DISPLAYPORT) // OSD over MSP; not supported (yet)    // не определено
        if (!osdDisplayPort) {
            osdDisplayPort = displayPortMspInit();
        }
#endif
        // osdInit  will register with CMS by itself.
        // osdInit сам зарегистрируется в CMS.
        // CMS (меню настроек) активировано через OSD или другой экран
        osdInit(osdDisplayPort);
    }
#endif

#if defined(USE_CMS) && defined(USE_SPEKTRUM_CMS_TELEMETRY) && defined(USE_TELEMETRY_SRXL) //- в common.h 
    // Register the srxl Textgen telemetry sensor as a displayport device
    // Регистрируем датчик телеметрии srxl Textgen как устройство displayport
    cmsDisplayPortRegister(displayPortSrxlInit());
#endif

#ifdef USE_GPS  //- в common.h
    if (feature(FEATURE_GPS)) { //фича не выставлена
        gpsInit();
    }
#endif


#ifdef USE_NAV //- в common.h
    navigationInit();   //инит навигации
#endif

#ifdef USE_LED_STRIP    //- в target.h
    ledStripInit(); // инит сид ленты (тупо false gl_variable)

    if (feature(FEATURE_LED_STRIP)) {   //фича не выставлена
        ledStripEnable();
    }
#endif

#ifdef USE_TELEMETRY //- в common.h
    if (feature(FEATURE_TELEMETRY)) { // фича - в target.h
        telemetryInit();
        /*
        инит телеметрии:
            FrSky
            LTM
            MAVLINK
            SIM
            CRSF
            SRXL
            **Все это в файле комон. Чисто под цель нет
        */
    }
#endif

#ifdef USE_BLACKBOX //- в common.h

    //Do not allow blackbox to be run faster that 1kHz. It can cause UAV to drop dead when digital ESC protocol is used
    // Не позволять черному ящику работать быстрее, чем 1 кГц. Это может привести к падению БЛА при использовании цифрового протокола ESC.
    const uint32_t blackboxLooptime =  getLooptime() * blackboxConfig()->rate_denom / blackboxConfig()->rate_num;
    // = время основного цикла * Знаменатель скорости ведения журнала черного ящика / Числитель скорости записи в черный ящик

    if (blackboxLooptime < 1000) {
        blackboxConfigMutable()->rate_num = 1;
        blackboxConfigMutable()->rate_denom = ceil(1000 / getLooptime());
    }

    // SDCARD and FLASHFS are used only for blackbox
    // Make sure we only init what's necessary for 
    /*
    // SDCARD и FLASHFS используются только для черного ящика
     // Убедитесь, что мы инициализируем только то, что необходимо для черного ящика
    */
    switch (blackboxConfig()->device) {
#ifdef USE_FLASHFS  //- не определенно в target.h
        case BLACKBOX_DEVICE_FLASH:
#ifdef USE_FLASH_M25P16     //- не определенно в target.h
            if (!flashDeviceInitialized) {
                flashDeviceInitialized = flashInit();
            }
#endif
            flashfsInit();
            break;
#endif

#ifdef USE_SDCARD   //- в target.h
        case BLACKBOX_DEVICE_SDCARD:    
            sdcardInsertionDetectInit();    // инит gpio для sd карты
            sdcard_init();  //sd card spi
            afatfs_init();  // init file system
            break;
#endif
        default:
            break;
    }

    blackboxInit(); //инициализировать черный ящик
#endif

    gyroStartCalibration(); //калибровка гироскопов

#ifdef USE_BARO     //- в target.h
    baroStartCalibration(); //калибровка барометра
#endif

#ifdef USE_PITOT        //- в common.h
    pitotStartCalibration();    //калибровка датчика скорости (трубки Пито)
#endif

#if defined(USE_VTX_CONTROL)    //- в common.h
    vtxControlInit();   // в vtx
    vtxCommonInit();    // функции - заглушки
    vtxInit();          // ничего не делают

#ifdef USE_VTX_SMARTAUDIO   //- в common.h
    vtxSmartAudioInit();        // поиск и инит сериал порта, инит звукового устройства с опред параметрами
#endif

#ifdef USE_VTX_TRAMP        //- в common.h
    vtxTrampInit();             // поиск и инит сериал порта, выбор мощности (600) и назначение указателям на функции определений функций
#endif

#ifdef USE_VTX_FFPV         //- в common.h
    vtxFuriousFPVInit();    // поиск и инит сериал порта, 
#endif

#endif // USE_VTX_CONTROL

    // Now that everything has powered up the voltage and cell count be determined.
    // Теперь, когда все подключено, можно определить напряжение и количество ячеек.
    if (feature(FEATURE_VBAT | FEATURE_CURRENT_METER)) // //- в target.h FEATURE_CURRENT_METER, FEATURE_VBAT
        batteryInit(); // инит батареи (установка значений глобальным переменным)

#ifdef USE_RCDEVICE     //- в common.h
    rcdeviceInit();         // поиск и открытие соответсвующего ком-порта
#endif // USE_RCDEVICE


#ifdef USE_DSHOT        //- в target.h
    initDShotCommands();    //  инит кольцевого буфера
    /*
    DShot — это новый протокол обмена данными между полетным контроллером (ПК) и регуляторами скорости (ESC)
    */
#endif

    // Latch active features AGAIN since some may be modified by init().
    // Снова фиксируем активные функции, так как некоторые из них могут быть изменены с помощью init().
    latchActiveFeatures();  // получение битов с фичами 
    motorControlEnable = true;

#ifdef USE_SECONDARY_IMU    //- в common.h
    secondaryImuInit();         // повторный инит инерциальных датчиков
#endif
    fcTasksInit();  // инит планировщика, создание задач <-------------вернись сюда потом

#ifdef USE_OSD //- в target.h
    if (feature(FEATURE_OSD) && (osdDisplayPort != NULL)) { // если есть фича OSD и дисплей порт
        setTaskEnabled(TASK_OSD, feature(FEATURE_OSD)); // добавить задачу
    }
#endif

#ifdef USE_RPM_FILTER   //- в common_post.h
    /*
    RPM фильтр - это фильтр для от помех, которые создаются моторами
    */
    disableRpmFilters(); //запрет фильтров (инит указателя на функцию null функцией)
    // если датчики движков с цифровым управлением разрешены И (
    // rpmFilterConfig()->gyro_filter_enabled - Включает фильтр оборотов гироскопа. Устанавливайте значение «ON» только тогда, 
    // когда телеметрия ESC работает и скорость вращения двигателей правильно передается в INAV
    //  ИЛИ
    // 
    if (STATE(ESC_SENSOR_ENABLED) && (rpmFilterConfig()->gyro_filter_enabled || rpmFilterConfig()->dterm_filter_enabled)) {
        rpmFiltersInit();   //инит фильтров (почему то одни гиро- фильтры)
        setTaskEnabled(TASK_RPM_FILTER, true);  // добавить задачу
    }
#endif

#ifdef USE_I2C_IO_EXPANDER  //- в common.h
    ioPortExpanderInit();       // инит расширителя порта pcf8574
#endif

#ifdef USE_POWER_LIMITS //- в common.h
    powerLimiterInit();   // инит режима экономии энергии  
#endif

    // Considering that the persistent reset reason is only used during init
    // Учитывая, что причина постоянного сброса используется только во время инициализации
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE); //запись в бэкап регистры 
    // "ПОСТОЯННАЯ ПРИЧИНА СБРОСА ОБЪЕКТА, СБРОСА НЕТ"

    systemState |= SYSTEM_STATE_READY; // состояние системы: готова
}

//tasks
[TASK_NUM] = {
    .taskName = "...",
    .taskFunc = ...,
    .desirePeriod = ...,    // частота
    .staticPriority = ...,  // приоритет
}

void fcTasksInit(void)
{
    schedulerInit();    //очистка очереди задач, добавление в очередь 0-го элемента массива со структурами задач
    /*
    Задача носит имя SYSTEM и вычисляет время загрузки
    */

    rescheduleTask(TASK_PID, getLooptime());    //перезапускаем задачу с номером TASK_PID и новым периодом

    // в функции есть проверка кто ее вызвал: она сама или какая-то другая задача из массива
    // в любом случае, если true и задача имеет функцию реализации - добавляет в очередь
    setTaskEnabled(TASK_PID, true);             

    rescheduleTask(TASK_GYRO, getGyroLooptime());
    setTaskEnabled(TASK_GYRO, true);

    setTaskEnabled(TASK_AUX, true);

    setTaskEnabled(TASK_SERIAL, true);
#if defined(BEEPER) || defined(USE_DSHOT)
    setTaskEnabled(TASK_BEEPER, true);
#endif
#ifdef USE_LIGHTS
    setTaskEnabled(TASK_LIGHTS, true);
#endif
    setTaskEnabled(TASK_BATTERY, feature(FEATURE_VBAT) || isAmperageConfigured());
    setTaskEnabled(TASK_TEMPERATURE, true);
    setTaskEnabled(TASK_RX, true);
#ifdef USE_GPS
    setTaskEnabled(TASK_GPS, feature(FEATURE_GPS));
#endif
#ifdef USE_MAG
    setTaskEnabled(TASK_COMPASS, sensors(SENSOR_MAG));
#if defined(USE_MAG_MPU9250)
    // fixme temporary solution for AK6983 via slave I2C on MPU9250
    rescheduleTask(TASK_COMPASS, TASK_PERIOD_HZ(40));
#endif
#endif
#ifdef USE_BARO
    setTaskEnabled(TASK_BARO, sensors(SENSOR_BARO));
#endif
#ifdef USE_PITOT
    setTaskEnabled(TASK_PITOT, sensors(SENSOR_PITOT));
#endif
#ifdef USE_RANGEFINDER
    setTaskEnabled(TASK_RANGEFINDER, sensors(SENSOR_RANGEFINDER));
#endif
#ifdef USE_DASHBOARD
    setTaskEnabled(TASK_DASHBOARD, feature(FEATURE_DASHBOARD));
#endif
#ifdef USE_TELEMETRY
    setTaskEnabled(TASK_TELEMETRY, feature(FEATURE_TELEMETRY));
#endif
#ifdef USE_LED_STRIP
    setTaskEnabled(TASK_LEDSTRIP, feature(FEATURE_LED_STRIP));
#endif
#ifdef STACK_CHECK
    setTaskEnabled(TASK_STACK_CHECK, true);
#endif
#if defined(USE_PWM_SERVO_DRIVER) || defined(USE_SERVO_SBUS)
    setTaskEnabled(TASK_PWMDRIVER, (servoConfig()->servo_protocol == SERVO_TYPE_SERVO_DRIVER) || (servoConfig()->servo_protocol == SERVO_TYPE_SBUS) || (servoConfig()->servo_protocol == SERVO_TYPE_SBUS_PWM));
#endif
#ifdef USE_CMS
#ifdef USE_MSP_DISPLAYPORT
    setTaskEnabled(TASK_CMS, true);
#else
    setTaskEnabled(TASK_CMS, feature(FEATURE_OSD) || feature(FEATURE_DASHBOARD));
#endif
#endif
#ifdef USE_OPFLOW
    setTaskEnabled(TASK_OPFLOW, sensors(SENSOR_OPFLOW));
#endif
#ifdef USE_VTX_CONTROL
#if defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP)
    setTaskEnabled(TASK_VTXCTRL, true);
#endif
#endif
#ifdef USE_RCDEVICE
    setTaskEnabled(TASK_RCDEVICE, rcdeviceIsEnabled());
#endif
#ifdef USE_PROGRAMMING_FRAMEWORK
    setTaskEnabled(TASK_PROGRAMMING_FRAMEWORK, true);
#endif
#ifdef USE_IRLOCK
    setTaskEnabled(TASK_IRLOCK, irlockHasBeenDetected());
#endif
#if defined(USE_SMARTPORT_MASTER)
    setTaskEnabled(TASK_SMARTPORT_MASTER, true);
#endif
#ifdef USE_SECONDARY_IMU
    setTaskEnabled(TASK_SECONDARY_IMU, secondaryImuConfig()->hardwareType != SECONDARY_IMU_NONE && secondaryImuState.active);
#endif
}

//задачи
cfTask_t cfTasks[TASK_COUNT] = {
    /*
    Если ожидающих задач больше 0 (totalWaitingTasksSamples > 0) - высчитывает средний процент загрузки системы:
        averageSystemLoadPercent = 100 * totalWaitingTasks / totalWaitingTasksSamples;
    Обнуляем общее количество образцов ожидающих задач и количество ожидающих задач:
    totalWaitingTasksSamples = 0;
    totalWaitingTasks = 0;
    */
    [TASK_SYSTEM] = {
        .taskName = "SYSTEM",
        .taskFunc = taskSystem,
        .desiredPeriod = TASK_PERIOD_HZ(10),              // run every 100 ms, 10Hz
        .staticPriority = TASK_PRIORITY_HIGH,
    },

    /*
    Высчитывается дельта времени между запусками текущей задачи
    При определенных условиях обновляет временные параметры
    Фильтрует гироскоп
    Обновляет акселерометр
    */
    [TASK_PID] = {
        .taskName = "PID",
        .taskFunc = taskMainPidLoop,
        .desiredPeriod = TASK_PERIOD_US(1000),
        .staticPriority = TASK_PRIORITY_REALTIME,
    },

    /* Обновление фактических показаний оборудования */
    [TASK_GYRO] = {
        .taskName = "GYRO",
        .taskFunc = taskGyro,
        .desiredPeriod = TASK_PERIOD_US(TASK_GYRO_LOOPTIME),
        .staticPriority = TASK_PRIORITY_REALTIME,
    },


    [TASK_SERIAL] = {
        .taskName = "SERIAL",
        .taskFunc = taskHandleSerial,
        .desiredPeriod = TASK_PERIOD_HZ(100),     // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
        .staticPriority = TASK_PRIORITY_LOW,
    },

#if defined(BEEPER) || defined(USE_DSHOT)
    [TASK_BEEPER] = {
        .taskName = "BEEPER",
        .taskFunc = beeperUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(100),     // 100 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_LIGHTS
    [TASK_LIGHTS] = {
        .taskName = "LIGHTS",
        .taskFunc = lightsUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(100),     // 100 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

    [TASK_BATTERY] = {
        .taskName = "BATTERY",
        .taskFunc = taskUpdateBattery,
        .desiredPeriod = TASK_PERIOD_HZ(50),      // 50 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_TEMPERATURE] = {
        .taskName = "TEMPERATURE",
        .taskFunc = taskUpdateTemperature,
        .desiredPeriod = TASK_PERIOD_HZ(100),     // 100 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },

    [TASK_RX] = {
        .taskName = "RX",
        .checkFunc = taskUpdateRxCheck,
        .taskFunc = taskUpdateRxMain,
        .desiredPeriod = TASK_PERIOD_HZ(50),      // If event-based scheduling doesn't work, fallback to periodic scheduling
        .staticPriority = TASK_PRIORITY_HIGH,
    },

#ifdef USE_GPS
    [TASK_GPS] = {
        .taskName = "GPS",
        .taskFunc = taskProcessGPS,
        .desiredPeriod = TASK_PERIOD_HZ(50),      // GPS usually don't go raster than 10Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_MAG
    [TASK_COMPASS] = {
        .taskName = "COMPASS",
        .taskFunc = taskUpdateCompass,
        .desiredPeriod = TASK_PERIOD_HZ(10),      // Compass is updated at 10 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_BARO
    [TASK_BARO] = {
        .taskName = "BARO",
        .taskFunc = taskUpdateBaro,
        .desiredPeriod = TASK_PERIOD_HZ(20),
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_PITOT
    [TASK_PITOT] = {
        .taskName = "PITOT",
        .taskFunc = taskUpdatePitot,
        .desiredPeriod = TASK_PERIOD_HZ(100),
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_RANGEFINDER
    [TASK_RANGEFINDER] = {
        .taskName = "RANGEFINDER",
        .taskFunc = taskUpdateRangefinder,
        .desiredPeriod = TASK_PERIOD_MS(70),
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_IRLOCK
    [TASK_IRLOCK] = {
        .taskName = "IRLOCK",
        .taskFunc = taskUpdateIrlock,
        .desiredPeriod = TASK_PERIOD_HZ(100),
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_DASHBOARD
    [TASK_DASHBOARD] = {
        .taskName = "DASHBOARD",
        .taskFunc = taskDashboardUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(10),
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_TELEMETRY
    [TASK_TELEMETRY] = {
        .taskName = "TELEMETRY",
        .taskFunc = taskTelemetry,
        .desiredPeriod = TASK_PERIOD_HZ(500),         // 500 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#if defined(USE_SMARTPORT_MASTER)
    [TASK_SMARTPORT_MASTER] = {
        .taskName = "SPORT MASTER",
        .taskFunc = taskSmartportMaster,
        .desiredPeriod = TASK_PERIOD_HZ(500),         // 500 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef USE_LED_STRIP
    [TASK_LEDSTRIP] = {
        .taskName = "LEDSTRIP",
        .taskFunc = taskLedStrip,
        .desiredPeriod = TASK_PERIOD_HZ(100),         // 100 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#if defined(USE_PWM_SERVO_DRIVER) || defined(USE_SERVO_SBUS)
    [TASK_PWMDRIVER] = {
        .taskName = "SERVOS",
        .taskFunc = taskSyncServoDriver,
        .desiredPeriod = TASK_PERIOD_HZ(200),         // 200 Hz
        .staticPriority = TASK_PRIORITY_HIGH,
    },
#endif

#ifdef STACK_CHECK
    [TASK_STACK_CHECK] = {
        .taskName = "STACKCHECK",
        .taskFunc = taskStackCheck,
        .desiredPeriod = TASK_PERIOD_HZ(10),          // 10 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef USE_OSD
    [TASK_OSD] = {
        .taskName = "OSD",
        .taskFunc = taskUpdateOsd,
        .desiredPeriod = TASK_PERIOD_HZ(250),
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_CMS
    [TASK_CMS] = {
        .taskName = "CMS",
        .taskFunc = cmsHandler,
        .desiredPeriod = TASK_PERIOD_HZ(50),
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_OPFLOW
    [TASK_OPFLOW] = {
        .taskName = "OPFLOW",
        .taskFunc = taskUpdateOpticalFlow,
        .desiredPeriod = TASK_PERIOD_HZ(100),   // I2C/SPI sensor will work at higher rate and accumulate, UART sensor will work at lower rate w/o accumulation
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_RCDEVICE
    [TASK_RCDEVICE] = {
        .taskName = "RCDEVICE",
        .taskFunc = rcdeviceUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(10),        // 10 Hz, 100ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#if defined(USE_VTX_CONTROL)
    [TASK_VTXCTRL] = {
        .taskName = "VTXCTRL",
        .taskFunc = vtxUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(5),          // 5Hz @200msec
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif
#ifdef USE_PROGRAMMING_FRAMEWORK
    [TASK_PROGRAMMING_FRAMEWORK] = {
        .taskName = "PROGRAMMING",
        .taskFunc = programmingFrameworkUpdateTask,
        .desiredPeriod = TASK_PERIOD_HZ(10),          // 10Hz @100msec
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif
#ifdef USE_SECONDARY_IMU
    [TASK_SECONDARY_IMU] = {
        .taskName = "IMU2",
        .taskFunc = taskSecondaryImu,
        .desiredPeriod = TASK_PERIOD_HZ(10),          // 10Hz @100msec
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif
#ifdef USE_RPM_FILTER
    [TASK_RPM_FILTER] = {
        .taskName = "RPM",
        .taskFunc = rpmFilterUpdateTask,
        .desiredPeriod = TASK_PERIOD_HZ(RPM_FILTER_UPDATE_RATE_HZ),          // 300Hz @3,33ms
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif
    [TASK_AUX] = {
        .taskName = "AUX",
        .taskFunc = taskUpdateAux,
        .desiredPeriod = TASK_PERIOD_HZ(TASK_AUX_RATE_HZ),          // 100Hz @10ms
        .staticPriority = TASK_PRIORITY_HIGH,
    },
};