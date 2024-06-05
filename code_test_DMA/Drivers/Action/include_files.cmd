@echo off
rem _________________________________________________________________________________________________________________________
rem ___________________________________________ Script écrit par Ronan Le Corronc ___________________________________________
rem ______________________________________ Permet d'ajouter les fichiers HAL manquant _______________________________________
rem __________________________________________________ Écrit le 31/05/2024 __________________________________________________
rem _________________________________________________________________________________________________________________________

rem Les variables ci-dessous sont à modifier pour les fichiers à ajouter et la version de HAL utilisée.
rem Le reste du script ne doit normalement pas être touché.

rem Version de HAL utilisé, à modifier en cas de mise à jour
set "HALversion=1.28.0"

rem Défini les types de fichier à ajouter au projet, à bien noter en minuscule
set "composant[0]=uart"
set "composant[1]=usart"
set "composant[2]=tim"

rem Défini le nombre de fichier à ajouter (lié directement aux variables d'au dessus)
set "nombreComposant=3"

rem Défini la vitesse de la clock externe (en Hz)
set "hse_clock_speed=8000000"

rem _________________________________________________________________________________________________________________________

rem Définit l'emplacement des librairies HAL
set "HALSrcPath=%USERPROFILE%\STM32Cube\Repository\STM32Cube_FW_F4_V%HALversion%\Drivers\STM32F4xx_HAL_Driver\Src\"
set "HALIncPath=%USERPROFILE%\STM32Cube\Repository\STM32Cube_FW_F4_V%HALversion%\Drivers\STM32F4xx_HAL_Driver\Inc\"

rem Définit les fichiers à traiter
set "filePath=..\..\Core\Inc\"

set "confFile=stm32f4xx_hal_conf.h"
set "tempFile=temp_hal_conf.h"

set "confPath=%filePath%%confFile%"
set "tempPath=%filePath%%tempFile%"

rem Efface le fichier de sortie s'il existe déjà
if exist "%tempPath%" del "%tempPath%"

rem Passe de nombreComposant à nombrComposant - 1 pour la lecture du tableau
set /a nombreComposant=%nombreComposant%-1

for /L %%i in (0 1 %nombreComposant%) do (

    setlocal enabledelayedexpansion

    rem Met en minuscule le nom des composants pour les fichiers à copier
    set composant=!composant[%%i]!
    call :CONV_VAR_to_min composant

    rem Ajoute les fichiers HAL au projet
    copy "%HALSrcPath%stm32f4xx_hal_!composant!.c" "..\STM32F4xx_HAL_Driver\Src"
    copy "%HALSrcPath%stm32f4xx_ll_!composant!.c" "..\STM32F4xx_HAL_Driver\Src"
    copy "%HALIncPath%stm32f4xx_hal_!composant!.h" "..\STM32F4xx_HAL_Driver\Inc"
    copy "%HALIncPath%stm32f4xx_ll_!composant!.h" "..\STM32F4xx_HAL_Driver\Inc"

    rem Met en majuscule le nom des composants pour les #define
    set composant=!composant[%%i]!
    call :CONV_VAR_to_MAJ composant

    rem Prépare les lignes à remplacer et celle qui remplace
    set "nouvelle_ligne=#define HAL_!composant!_MODULE_ENABLED"
    set "ligne_a_remplacer=/* !nouvelle_ligne! */"

    rem Désactiver l'expansion retardée pour la lecture des lignes du fichier
    rem cela permet de conserver tous les caractères (en partie le "!" qui est habituellement détécté comme une commande)
    setlocal DisableDelayedExpansion

    rem Parcourir chaque ligne du fichier d'entrée
    for /f "tokens=1* delims=:" %%a in ('findstr /n "^" "%confPath%"') do (

        rem Lis la ligne suivante du fichier conf.h
        set "ligne=%%b"

        rem Réactive l'expansion retardée pour pouvoir utiliser les variables précédemment déclarées
        setlocal EnableDelayedExpansion

        rem Comparer la ligne courante avec la ligne à remplacer        
        if "!ligne!"=="!ligne_a_remplacer!" (
            rem Si la ligne correspond, écrire la nouvelle ligne
            >> "%tempPath%" echo !nouvelle_ligne!

        ) else (
            rem Sinon, écrire la ligne courante inchangée
            >> "%tempPath%" echo(!ligne!
        )

        rem Nettoye les variables globales pour le rebouclage
        endlocal
    )

    rem Supprime le fichier conf.h original par le fichier temporaire modifié
    del "%confPath%"
    
    rem Renomme le fichier temporaire en fichier conf.h pour qu'il puisse être utilisé par STMCubeIDE comme le fichier original
    rename "%tempPath%" "%confFile%"

)

rem Re-définition de la vitesse de la clock externe
set "hse_clock_speed_define=#define HSE_VALUE"
set "hse_clock_speed_8=  #define HSE_VALUE    %hse_clock_speed%U"

setlocal DisableDelayedExpansion

for /f "tokens=1* delims=:" %%a in ('findstr /n "^" "%confPath%"') do (

    rem Lis la ligne suivante du fichier conf.h
    set "ligne=%%b"

    setlocal enabledelayedexpansion

    rem Récupère les deux premiers mots de la ligne actuelle
    for /f "tokens=1,2 delims= " %%a in ("!ligne!") do (
        set "define_ligne=%%a %%b"
    )

    rem Vérifie si il est sur la ligne ou la vitesse de la clock est définie
    if "!define_ligne!"=="!hse_clock_speed_define!" (
        rem Si c'est le cas, il re-écrit cette ligne avec la vitesse souhaitée
        >> "%tempPath%" echo !hse_clock_speed_8!

    ) else (
        rem Sinon il écrit la ligne lue
        >> "%tempPath%" echo(!ligne!
    )

    endlocal
)

rem Supprime le fichier conf.h original par le fichier temporaire modifié
del "%confPath%"

rem Renomme le fichier temporaire en fichier conf.h pour qu'il puisse être utilisé par STMCubeIDE comme le fichier original
rename "%tempPath%" "%confFile%"

rem Efface le fichier de sortie si jamais il existe toujours (normalement pas executé)
if exist "%tempPath%" del "%tempPath%"






















EXIT /b 0

rem Deux fonctions permettant de passer un string en majuscule ou en minuscule. Utile pour les #define
:CONV_VAR_to_MAJ
    FOR %%z IN (A B C D E F G H I J K L M N O P Q R S T U V W X Y Z) DO CALL set %~1=%%%~1:%%z=%%z%%
    EXIT /b 0

:CONV_VAR_to_min
    FOR %%z IN (a b c d e f g h i j k l m n o p q r s t u v w x y z) DO CALL set %~1=%%%~1:%%z=%%z%%
    EXIT /b 0