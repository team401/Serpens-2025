import java.io.File
import java.time.Year
import kotlin.system.exitProcess

//Pull in git submodle
Runtime.getRuntime().exec("git submodule add https://github.com/team401/Vektor")
println("[project_setup] Added Vektor git submodule")

//Retrieve project name from folder
val folderName = File(System.getProperty("user.dir")).name
val folderNameSplit = folderName.split('-')
val year = folderNameSplit.getOrNull(0)?.toIntOrNull() ?: 0
val name = folderNameSplit.getOrElse(1) { "" }.toLowerCase()

if (year == 0) {
    println("No valid year found in project name.  Please fix your folder and repo name to 'YEAR-PROJECTNAME-Code'")
    exitProcess(1)
}

if (name.isBlank()) {
    println("No valid project name found.  Please fix your folder and repo name to 'YEAR-PROJECTNAME-Code'")
    exitProcess(1)
}

println("[project_setup] Identified project year '$year', project name '$name'")

val settingsDotGradle = File("settings.gradle")
settingsDotGradle.writeText(
    settingsDotGradle.readText()
        .replace("Robot-Code-Template", folderName)
)
val settingsText = settingsDotGradle.readText()
if (!settingsText.contains("includeBuild 'Vektor'")) {
    settingsDotGradle.writeText(
            settingsText + "\n\nincludeBuild 'Vektor'"
    )
}

println("[project_setup] Updated settings.gradle")

val buildDotGradle = File("build.gradle")
buildDotGradle.writeText(
    buildDotGradle.readText()
        .replace("YEAR_REPLACE", year.toString())
        .replace("PROJECT_REPLACE", name)
        .replace("//UNCOMMENT", "")
)
println("[project_setup] Updated build.gradle")

val sourceDir = File("src/main/kotlin/org/team401/PROJECT_YEAR")
val renamed = File("src/main/kotlin/org/team401/$name$year")
sourceDir.renameTo(renamed)
val main = File("src/main/kotlin/org/team401/$name$year/Main.kt")
main.writeText(
        main.readText()
                .replace("PROJECT_YEAR", "$name$year")
)

println("\u001B[33m" + "[project_setup] Done.  Please update 'build.gradle' with the latest versions of FRC libraries.")