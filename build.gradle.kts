plugins {
   id("us.ihmc.ihmc-build") version "0.20.2"
   id("us.ihmc.ihmc-ci") version "5.9"
   id("us.ihmc.ihmc-cd") version "1.14"
}

ihmc {
   group = "us.ihmc"
   version = "0.7.0"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-yovariables"
   openSource = true
   maintainer = "Georg Wiedebach (gwiedebach@ihmc.us)"
   maintainer = "Sylvain Bertrand (sbertrand@ihmc.us)"

   configureDependencyResolution()
   configurePublications()
}

dependencies {
   api("org.apache.commons:commons-math3:3.3")
   api("org.apache.commons:commons-lang3:3.9")
   api("net.sf.trove4j:trove4j:3.0.3")
   api("jakarta.xml.bind:jakarta.xml.bind-api:2.3.2")
   api("org.glassfish.jaxb:jaxb-runtime:2.3.2")

   api("us.ihmc:ihmc-commons:0.30.0")
   api("us.ihmc:euclid-frame:0.15.0")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.30.0")
   api("us.ihmc:euclid-test:0.15.0")
}
