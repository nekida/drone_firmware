## Основы
Файл должен иметь название "CMakeLists.txt"

"#" - символ комментария

Сборку можно производить в другой папке, передав путь к исходним и файлу .txt через команду. Или создать папку "build" и сделать в ней "cmake .."

Если CMakeLists.txt был изменен, то вызов make автоматически запустит cmake. Если исходники были перемещены, то нужно очистить временную директорию и запустить cmake вручную.

```cmake_minimum_required(VERSION 2.8)``` - Проверка версии CMake. Если версия установленой программы старее указаной, произойдёт аварийный выход.

```add_executable(main main.cpp)``` - Создает исполняемый файл с именем main из исходника main.cpp. Расширение исполняемого файла зависит от целевой платформы (.exe для Windows)

```project(projectName)``` - название проекта

```set(VARIABLE_NAME main.cpp)``` - Установка переменной "VARIABLE_NAME" со списком исходников для исполняемого файла. Переменные могут хранить списки значений, разделённых пробелами\табуляциями\переносами:

```set(SOURCE main.cpp foo.cpp)```

```set(HEADER main.h```

```foo.h)```

Оба варианта правильные.


```message("Hello, my lovely CMake!")``` - напечатает "Hello, my lovely CMake!", ```message(Hello, my lovely CMake!)``` - напечатает "Hello,mylovelyCMake!" (предыдущую строку без пробелов).

Двойные кавычки позволяют превратить список в строку (разделенную символом ";"): 

```set(ARGUMENT Walk around the forest)```

```message("${ARGUMENT}")``` - напечатает ```Walk;around;the;forest```

```message(${ARGUMENT})``` - напечатает ```Walkaroundtheforest```

Подстановку переменной любого типа не в контексте строковой константы — превращает ее автоматически в список всегда. символы ";" исчезают:

```set(ARGUMENT "Walk around the forest")```

```message("${ARGUMENT}")``` - напечатает ```Walk around the forest```, рассматривает как строку, ```set(ARGUMENT "Walk;around;the;forest")``` напечатает как ```Walk;around;the;forest```

```message(${ARGUMENT})``` - напечатает ```Walk around the forest```, ```set(ARGUMENT "Walk;around;the;forest")``` напечатает как ```Walkaroundtheforest```

Добавление в двойные ковычки "" переменной, которая и так строка, делает из нее строку, содержащую любые символы и знаки, а переменная-список при печати в двойных ковычках будет иметь ";" между своими членами. Печать списка без ковычек выведет обычный список без пробелов между его элементами.

|                           |set(ARGUMENT Walk around the forest) |set(ARGUMENT "Walk around the forest") |set(ARGUMENT "Walk;around;the;forest") |
|:-------------------------:|:-----------------------------------:|:-------------------------------------:|:-------------------------------------:|
|message("${ARGUMENT}")     |Walk;around;the;forest               |Walk around the forest                 |Walk;around;the;forest
|message(${ARGUMENT})       |Walkaroundtheforest                  |Walk around the forest                 |Walkaroundtheforest

Следует запомнить, что практически все API — все эти if-ы, foreach и прочие add_target — работают со списком строк. т.е. как бы ```vector<string>```
соответственно, можно делать штуки вроде ```if (${some_long_condition})``` и отсюда приходит понимание почему иногда выдает ошибку код ```if (${foo} STREQUAL "bar")``` — если ```foo``` неопределена (или пуста), условие превратится в список из двух строк «STREQUAL» и «bar». Поэтому следует понять при чтении документации, когда допустимо значение, когда строка, а когда и то, и то (и это в документации обычно явно отражено фразами вроде «string or variable»)

```target_sources(MyExecutable MessageConsumer.c)``` - добавляет к цели "MyExecutable" исходник "MessageConsumer.c"

```unset(VARIABLE_NAME)``` - удалить переменную VARIABLE_NAME

```add_library(lib_name STATIC ${SOURCE_LIB})``` - Создание статической библиотеки с именем "lib_name" из списка исходников с именем переменной "SOURCE_LIB". Чтобы получить значение переменной: 
```${var_name}```. "STATIC" дает статическую библиотеку, "SHARED" - динамическую. Если тип библиотеки не указать, по умолчанию она соберётся как статическая.

```add_library(netutil STATIC IMPORTED)``` - Слово «IMPORTED», указывает, что библиотека берётся извне.

Окончательное имя библиотеки зависит от целевой платформы (например, lib<LibraryName>.a или <LibraryName>.lib).
  
- Статические библиотеки задаются ключевым словом STATIC вторым аргументом и представляют из себя архивы объектных файлов, связываемых с исполняемыми файлами и другими библиотеками во время компиляции;
- Динамические библиотеки задаются ключевым словом SHARED вторым аргументом и представляют из себя двоичные библиотеки, загружаемые операционной системой во время выполнения программы;
- Модульные библиотеки задаются ключевым словом MODULE вторым аргументом и представляют из себя двоичные библиотеки, загружаемые посредством техник выполнения самим исполняемым файлом;
- Объектные библиотеки задаются ключевым словом OBJECT вторым аргументом и представляют из себя набор объектных файлов, связываемых с исполняемыми файлами и другими библиотеками во время компиляции.

```set_property(TARGET netutil PROPERTY```

```       IMPORTED_LOCATION Binary/game_client/libnetutil.a)``` - В CMake каждая цель имеет параметры, а set_property позволяет их изменять.

```target_link_libraries(${TARGET} netutil)``` - Линкование внешней библиотеки. Для динамических библиотек все аналогично, только тип "SHARED", расширение — ".so".

```target_link_libraries(main lib_name)``` - Линковка программы с библиотекой

```include_directories(foo)``` - Расположение заголовочных файлов. Может быть вызвана несколько раз. Хидеры будут искаться во всех указаных директориях.

```add_subdirectory(foo)``` - Добавление подпроекта, указывается имя дирректории

```message("CMake the running...\n")``` - Печать сообщения "CMake the running...\n" в консоль

```option(USE_ANOTHER_LIBRARY "Do you want to use an another library?" OFF)``` - задать опцию "USE_ANOTHER_LIBRARY" с описанием "Do you want to use an another library?" и значением "OFF"

## Возможности
### Условный оператор

Напечатает "Of course, 5 > 1!":

```
if(5 GREATER 1)
    message("Of course, 5 > 1!")
elseif(5 LESS 1)
    message("Oh no, 5 < 1!")
elseif(5 EQUAL 3)
    message("Oh no, 5 != 3!")
else()
    message("Oh my god, 5 == 1!")
endif() 
```

### Циклы

-Напечатает в консоль три раза "VARIABLE is still 'Airport'":

```
set(VARIABLE Airport)
while(${VARIABLE} STREQUAL Airport)
    message("VARIABLE is still '${VARIABLE}'")
    message("VARIABLE is still '${VARIABLE}'")
    message("VARIABLE is still '${VARIABLE}'")
    set(VARIABLE "Police station")
endwhile()
```


-На каждой итерации данного цикла переменной VARIABLE присваивается следующее значение из списка Give me the sugar please!, а затем исполняется команда message(${VARIABLE}), которая выводит текущее значение переменной VARIABLE. Когда значений в списке не остаётся, то цикл завершает своё выполнение. Команда endforeach сигнализирует о завершении списка вложенных в цикл команд.
Напечатает "Give me the sugar please!" с новых строк:

```
foreach(VARIABLE Give me the sugar please!)
    message(${VARIABLE})
endforeach()
```

Существуют ещё 3 формы записи цикла foreach. Первый цикл в данном примере на место списка генерирует целые числа от 0 до 10, второй цикл генерирует в диапазоне от 3 до 15, а третий цикл работает в сегменте от 50 до 90, но с шагом 10.

```
# Напечатает "0 1 2 3 4 5 6 7 8 9 10" с новых строк:
foreach(VARIABLE RANGE 10)
    message(${VARIABLE})
endforeach()

# Напечатает "3 4 5 6 7 8 9 10 11 12 13 14 15" с новых строк:
foreach(VARIABLE RANGE 3 15)
    message(${VARIABLE})
endforeach()

# Напечатает "50 60 70 80 90" с новых строк:
foreach(VARIABLE RANGE 50 90 10)
    message(${VARIABLE})
endforeach()
```

### Функции и макросы

Синтаксис CMake позволяет определять собственные команды, которые можно будет вызывать в точности как встроенные. Приведённый ниже пример демонстрирует использование функций и макросов: сначала определяются функция и макрос со своими собственными командами, а при их вызове их команды исполняются последовательно.

```
# Определение функции "print_numbers":
function(print_numbers NUM1 NUM2 NUM3)
    message(${NUM1} " " ${NUM2} " " ${NUM3})
endfunction()

# Определение макроса "print_words":
macro(print_words WORD1 WORD2 WORD3)
    message(${WORD1} " "  ${WORD2} " " ${WORD3})
endmacro()

# Вызов функции "print_numbers", которая напечатает "12 89 225":
print_numbers(12 89 225)

# Вызов макроса "print_words", который напечатает "Hey Hello Goodbye":
print_words(Hey Hello Goodbye)
```

Поведение функций аналогично поведения в языке Си, макросы не имеют собственной области видимости и все их переменные глобальные.

```
# Определить макрос, содержащий команду выхода:
macro(demonstrate_macro)
    return()
endmacro()

# Определить функцию, вызывающую предыдущий макрос:
function(demonstrate_func)
    demonstrate_macro()
    message("The function was invoked!")
endfunction()

# Напечатает "Something happened with the function!"
demonstrate_func()
message("Something happened with the function!")
```

В приведённом выше примере функция demonstrate_func не успеет напечатать сообщение The function was invoked!, так как прежде, на место вызова макроса demonstrate_macro будет подставлена и выполнена команда выхода.

### cmake_parse_arguments
Мощный механизм cmake_parse_arguments позволяет производить разбор аргументов, переданных в функцию или макрос.


Данная команда принимает префикс, используемый в определении переменных (смотреть следующий абзац), список опций, используемых без последующих значений, список ключевых слов, после которых следует одно значение, список ключевых слов, после которых следуют множества значений и список всех значений, переданных в функцию или макрос.


Работа механизма разбора аргументов заключается в преобразовании полученных аргументов в значения переменных. Таким образом, рассмотренная команда для каждой опции и ключевого слова определяет собственную переменную вида <Prefix>_<OptionOrKeyword>, инкапсулирующую некоторое значение. Для опций — это булевы значения (истина — опция указана, иначе — ложь), а для ключевых слов — все переданные значения, расположенные после них.


Функция custom_function содержит вызов команды cmake_parse_arguments, а затем команды печати значений определённых переменных. Далее, функция вызывается с аргументами LOW NUMBER 30 COLORS red green blue, после чего производится печать на экран:

```
function(custom_function)
    # Вызвать механизм обработки аргументов для текущей функции:
    cmake_parse_arguments(CUSTOM_FUNCTION "LOW;HIGH" "NUMBER" "COLORS" ${ARGV})

    # Напечатает "'LOW' = [TRUE]":
    message("'LOW' = [${CUSTOM_FUNCTION_LOW}]")
    #Напечатает "'HIGH' = [FALSE]":
    message("'HIGH' = [${CUSTOM_FUNCTION_HIGH}]")
    # Напечатает "'NUMBER' = [30]":
    message("'NUMBER' = [${CUSTOM_FUNCTION_NUMBER}]")
    # Напечатает "'COLORS' = [red;green;blue]":
    message("'COLORS' = [${CUSTOM_FUNCTION_COLORS}]")
endfunction()

# Вызвать функцию "custom_function" с произвольными аргументами:
custom_function(LOW NUMBER 30 COLORS red green blue)
```

### Области видимости
Переменные можно определять в "родительской" области видимости с помощью команды set(VARIABLE ... PARENT_SCOPE). Данный пример демонстрирует эту особенность:
  
```
# Функция, определяющая переменную "VARIABLE" со значением
# "In the parent scope..." в родительской области видимости:
function(demonstrate_variable)
    set(VARIABLE "In the parent scope..." PARENT_SCOPE)
endfunction()

# Определить переменную "VARIABLE" в текущей области видимости:
demonstrate_variable()

# Теперь возможно получить к переменной "VARIABLE" доступ:
message("'VARIABLE' is equal to: ${VARIABLE}")
```
  
  Если из определения переменной VARIABLE убрать PARENT_SCOPE, то переменная будет доступна лишь функции demonstrate_variable, а в глобальной области видимости она примет пустое значение.
  
## Оформление и настройка
  
  ```
  # Задать характеристики проекта "MyProject":
project(MyProject VERSION 1.2.3.4 LANGUAGES C CXX)
  ```
  Стоит отметить, что если ключевое слово LANGUAGES опущено, то по умолчанию задаются языки C CXX. Вы также можете отключить указание любых языков путём написания ключевого слова NONE в качестве списка языков или просто оставить пустой список.
  
  Команда include заменяет строку своего вызова кодом заданного файла, действуя аналогично препроцессорной команде include языков C/C++. Этот пример запускает скриптовый файл MyCMakeScript.cmake описанной командой:
  ```
  message("'TEST_VARIABLE' is equal to [${TEST_VARIABLE}]")

# Запустить скрипт `MyCMakeScript.cmake` на выполнение:
include(MyCMakeScript.cmake)

message("'TEST_VARIABLE' is equal to [${TEST_VARIABLE}]")
  ```
  
  В данном примере, первое сообщение уведомит о том, что переменная TEST_VARIABLE ещё не определена, однако если скрипт MyCMakeScript.cmake определит данную переменную, то второе сообщение уже будет информировать о новом значении тестовой переменной. Таким образом, скриптовый файл, включаемый командой include, не создаёт собственной области видимости.
  
### Работа с целями
Имеется возможность управления свойствами целей, предназначенных для задания процесса сборки проекта. Команда get_target_property присваивает предоставленной переменной значение свойства цели. Данный пример выводит значение свойства C_STANDARD цели MyTarget на экран:
      
```
# Присвоить переменной "VALUE" значение свойства "C_STANDARD":
get_target_property(VALUE MyTarget C_STANDARD)

# Вывести значение полученного свойства на экран:
message("'C_STANDARD' property is equal to [${VALUE}]")
```
      
Команда set_target_properties устанавливает указанные свойства целей заданными значениями. Данная команда принимает список целей, для которых будут установлены значения свойств, а затем ключевое слово PROPERTIES, после которого следует список вида <название свойства> <новое значение>:
```
# Установить свойству 'C_STANDARD' значение "11",
# а свойству 'C_STANDARD_REQUIRED' значение "ON":
set_target_properties(MyTarget PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)
```
Пример выше задал цели MyTarget свойства, влияющие на процесс компиляции, а именно: при компиляции цели MyTarget CMake затребует компилятора о использовании стандарта C11.

Также имеется возможность проверки ранее определённых целей с помощью конструкции if(TARGET <TargetName>):
```
# Выведет "The target was defined!" если цель "MyTarget" уже определена,
# а иначе выведет "The target was not defined!":
if(TARGET MyTarget)
    message("The target was defined!")
else()
    message("The target was not defined!")
endif()
```
      
### Способы включения заголовков
      
Информировать компилятора о располжении включаемых заголовков можно посредством двух команд: include_directories и target_include_directories. Вы решаете, какую из них использовать, однако стоит учесть некоторые различия между ними (идея предложена в комментариях).


Команда include_directories влияет на область каталога. Это означает, что все директории заголовков, указанные данной командой, будут применяться для всех целей текущего CMakeLists.txt, а также для обрабатываемых подпроектов (смотреть add_subdirectory).


Команда target_include_directories влияет лишь на указанную первым аргументом цель, а на другие цели никакого воздействия не оказывается. Пример ниже демонстрирует разницу между этими двумя командами:
```
add_executable(RequestGenerator RequestGenerator.c)
add_executable(ResponseGenerator ResponseGenerator.c)

# Применяется лишь для цели "RequestGenerator":
target_include_directories(RequestGenerator headers/specific)

# Применяется для целей "RequestGenerator" и "ResponseGenerator":
include_directories(headers)
```
      
В комментариях упомянуто, что в современных проектах применение команд include_directories и link_libraries является нежелательным. Альтернатива — это команды target_include_directories и target_link_libraries, действующие лишь на конкретные цели, а не на всю текущую область видимости.
      
### Установка проекта
Команда install генерирует установочные правила для Вашего проекта. Данная команда способна работать с целями, файлами, папками и многим другим. Сперва рассмотрим установку целей.


Для установки целей необходимо первым аргументом описанной функции передать ключевое слово TARGETS, за которым должен следовать список устанавливаемых целей, а затем ключевое слово DESTINATION с расположением каталога, в который установятся указанные цели. Данный пример демонстрирует типичную установку целей:
```
# Установить цели "TimePrinter" и "DataScanner" в директорию "bin":
install(TARGETS TimePrinter DataScanner DESTINATION bin)
```
      
Процесс описания установки файлов аналогичен, за тем исключением, что вместо ключевого слова TARGETS следует указать FILES. Пример, демонстрирующий установку файлов:
```
# Установить файлы "DataCache.txt" и "MessageLog.txt" в директорию "~/":
install(FILES DataCache.txt MessageLog.txt DESTINATION ~/)
```
      
Процесс описания установки папок аналогичен, за тем исключением, что вместо ключевого слова FILES следует указать DIRECTORY. Важно подметить, что при установке будет копироваться всё содержимое папки, а не только её название. Пример установки папок выглядит следующим образом:
```
# Установить каталоги "MessageCollection" и "CoreFiles" в директорию "~/":
install(DIRECTORY MessageCollection CoreFiles DESTINATION ~/)
```
После завершения обработки CMake всех Ваших файлов Вы можете выполнить установку всех описанных объектов командой sudo checkinstall (если CMake генерирует Makefile), или же выполнить данное действие интегрированной средой разработки, поддерживающей CMake.
      
# More
[One](https://habr.com/ru/post/431428/)
      
[Two](https://habr.com/ru/post/432096/)
      
[Three](https://habr.com/ru/post/433822/)
