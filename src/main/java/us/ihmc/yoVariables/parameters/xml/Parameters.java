/*
 * Copyright 2017 Florida Institute for Human and Machine Cognition (IHMC)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package us.ihmc.yoVariables.parameters.xml;

import java.util.List;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;

import us.ihmc.yoVariables.parameters.XmlParameterReader;
import us.ihmc.yoVariables.parameters.XmlParameterWriter;
import us.ihmc.yoVariables.parameters.YoParameter;

/**
 * The class represents a XML root token used together with {@link XmlParameterReader} and
 * {@link XmlParameterWriter} to export and import {@link YoParameter} with their values to and from
 * XML files.
 */
@XmlRootElement
@XmlAccessorType(XmlAccessType.NONE)
public class Parameters
{
   /** The list of registries associated to this XML root token. */
   @XmlElement(name = "registry")
   private List<Registry> registries;

   /**
    * Returns the list of registries associated to this XML root token.
    * 
    * @return the list of registries.
    */
   public List<Registry> getRegistries()
   {
      return registries;
   }

   /**
    * Sets the list of registries associated to this XML root token.
    * 
    * @param registries the list of registries.
    */
   public void setRegistries(List<Registry> registries)
   {
      this.registries = registries;
   }
}
