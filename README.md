# ![YoVariables](logo/YoVariables.png)
[ ![ihmc-yovariables](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-yovariables/badge.svg?style=plastic)](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-yovariables)
![buildstatus](https://github.com/ihmcrobotics/ihmc-yovariables/actions/workflows/gradle-test.yml/badge.svg)

## What is YoVariables
YoVariables provides a framework for organizing, storing, and manipulating variables used in algorithms.
The variables are organized in a tree structure using registries.

This library was designed with the idea that a user should be able to:
- retrieve any control variable by name,
- list the variables declared by an algorithm,
- observe their value changing over time and have access to the variable history,
- should be able to tune a variable on the fly.
YoVariables is the foundation block to implement these features in the Simulation Construction Set project.

## Who would use YoVariables
Any developer using the Simulation Construction Set project for visualizing an algorithm.

Any developer who wants to organize control variables.

## What is the goal of YoVariables?
The objective is to provide a data structure that is easy to implement and intuitive to use and usable in a real-time environment.

## Content
### `YoRegistry` and `YoVariable`
A `YoVariable` is an abstract class that represents a primitive that has a name and can be registered in a `YoRegistry`. Five implementations can be found:
- `YoBoolean` to implement a variable holding a boolean value.
- `YoDouble` to implement a variable holding a double value.
- `YoInteger` to implement a variable holding an integer value.
- `YoLong` to implement a variable holding a long value.
- `YoEnum` to implement a variable holding an enum value.

`YoRegistry` represents a registry that contains variables and child registries.

### `YoParameter`
A `YoParameter` can be seen as a read-only `YoVariable` that guarantees that the algorithm declaring it will only read the parameter value. As a consequence, when dealing with a `YoParameter` on the user side, for instance in SCS, it is ensured that the parameter is only modified by the user.

A `YoParameter` can be attributed an initial value at construction or can also be loaded from an XML parameter file. See `XmlParameterReader` and `XmlParameterWriter` from more details.

### `YoBuffer`
`YoBuffer` provides an implementation of a buffer which can be used to store value history of collection of `YoVariable`s.

### `Filters`
The `Filters` project provides a way of generating yo variables with diffeerent types of filters on the resulting output signal. This includes:
- `AlphaFilteredYoVariable`, which applies an alpha filter to a `YoDouble`, which is equivalent to a low-pass filter.
- `RateLimitedYoVariable`, which limits the output of a `YoDouble` to change less than a cerrtain rate.
- `BacklashCompensatedVelocityYoVariable`, which attempts to remove the backlash from the velocity that is estimated from a finite-differenced position signal.
- `GlitchFilteredYoBoolean`, which forces a system to change value for a certain number of times before the return changes value.

## Using YoVariables from .jar releases with Maven/Gradle
The releases .jars for YoVariables are hosted on Bintray.
You can browse the IHMC release packages at https://bintray.com/ihmcrobotics/maven-release.
Instructions for adding the Maven repository and identifying the artifacts can also be found on Bintray for each package.

At a minimum, you will need to have the following repository declared in your build script to use the YoVariables .jars:

```gradle
repositories {
   mavenCentral()
}

dependencies {
   compile group: "us.ihmc", name: "ihmc-yovariables", version: "x.x",
   compile group: "us.ihmc", name: "ihmc-yovariables-filters", version: "x.x"
}
```
