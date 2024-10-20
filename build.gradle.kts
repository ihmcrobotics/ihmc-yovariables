plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
}

ihmc {
   group = "us.ihmc"
   version = "0.12.2"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-yovariables"
   openSource = true
   maintainer = "Sylvain Bertrand (sbertrand@ihmc.us)"

   configureDependencyResolution()
   configurePublications()
}

dependencies {
   api("net.sf.trove4j:trove4j:3.0.3")
   api("com.sun.xml.bind:jaxb-impl:4.0.5")

   api("us.ihmc:ihmc-commons:0.32.0")
   api("us.ihmc:euclid-frame:0.21.0")
}

filtersDependencies {
   api(ihmc.sourceSetProject("main"))
   api("org.ejml:ejml-ddense:0.39")
//   api("us.ihmc:ihmc-commons-utils:source")
}

testDependencies {
   api(ihmc.sourceSetProject("filters"))
   api("us.ihmc:ihmc-commons-testing:0.32.0")
   api("us.ihmc:euclid-test:0.21.0")
   api("org.apache.commons:commons-math3:3.3")
}
