# lib2960

**<ins>NOTE: This is a work in progress and has not been fully tested at this time.</ins>**

This is a library created by FRC Team 2960 Automation Nation to contain commonly used robot code.

The library is broken up by required vendor library. This is to keep unnecessary vendor libraries from being included when the part of this library that used it is not being used.

This is the core library. This submodule must be included for all of the vendor specific portions of the library to work correctly.

## Prerequisites  
The following is required for this library to work correctly
- [PathPlannerLib](https://pathplanner.dev/pplib-getting-started.html)
- [PhotonLib](https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html)

## Usage
This library is intended to be included as a Git submodule in a sub folder of a java based FRC robot project. 

Use the following steps to include this library:

1. Open your existing robot java project
2. Open the terminal by pressing ```ctrl + ` ```
3. If you do not already have a Git repository setup, initialize a git repository for your project
    - This can be done using the source control interface in VSCode
    - This can also be down by running the following commands in a terminal
        ```
        git init
        git add --all
        git commit "Initializing project repository"
        ```

4. Add this library as a submodule using the following commands in the terminal


    ```
    git submodule add https://github.com/Team-2960/lib2960.git src\main\java\frc\lib2960
    cd src\main\java\frc\lib2960
    git checkout tags/v2025.0.0-pre1
    cd ..\..\..\..\..
    ```

Once included, all git pull and clone operations must include the ```--recuse-submodules``` parameter to insure the submodule is also cloned with the main repository.

## Related Libraries
The lib2960 library is broken up by the required vendor libraries. The other associated lib2960 libraries include:

- lib2960 Core Library - [lib2960](https://github.com/Team-2960/lib2960)
  - This is the core library and is required for any of the vendor specific version to work
- lib2960 Cross the Road Electronics (CTRE) - [lib2960_ctre](https://github.com/Team-2960/lib2960_ctre)
  - Includes implementations of lib2960 classes that use the CTRE Phoenix vendor libraries
- lib2960 Rev - [lib2960_rev](https://github.com/Team-2960/lib2960_rev)
  - Includes implementations of lib2960 classes that use the RevLib vendor libraries