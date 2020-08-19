plugins {
   id("us.ihmc.ihmc-build") version "0.21.0"
   id("us.ihmc.ihmc-ci") version "5.9"
   id("us.ihmc.ihmc-cd") version "1.14"
}

ihmc {
   group = "us.ihmc"
   version = "0.9.2"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-yovariables"
   openSource = true
   maintainer = "Georg Wiedebach (gwiedebach@ihmc.us)"
   maintainer = "Sylvain Bertrand (sbertrand@ihmc.us)"

   configureDependencyResolution()
   configurePublications()
}

dependencies {
   api("net.sf.trove4j:trove4j:3.0.3")
   api("jakarta.xml.bind:jakarta.xml.bind-api:2.3.2")
   api("org.glassfish.jaxb:jaxb-runtime:2.3.2")

   api("us.ihmc:ihmc-commons:0.30.2")
   api("us.ihmc:euclid-frame:0.15.0")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.30.2")
   api("us.ihmc:euclid-test:0.15.0")
}
