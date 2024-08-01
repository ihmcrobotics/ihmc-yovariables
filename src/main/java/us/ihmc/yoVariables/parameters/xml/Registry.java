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

import java.util.ArrayList;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;

import us.ihmc.yoVariables.parameters.XmlParameterReader;
import us.ihmc.yoVariables.parameters.XmlParameterWriter;
import us.ihmc.yoVariables.parameters.YoParameter;

/**
 * This class represents a XML token used together with {@link XmlParameterReader} and
 * {@link XmlParameterWriter} to export and import {@link YoParameter} with their values to and from
 * XML files.
 * <p>
 * This XML token represents a registry which can have sub-registries and parameters.
 * </p>
 */
@XmlAccessorType(XmlAccessType.NONE)
public class Registry
{
   /** The name of this registry. */
   @XmlAttribute
   private String name;

   /** The children of this registry. */
   @XmlElement(name = "registry")
   private List<Registry> registries;

   /** The parameters in this registry. */
   @XmlElement(name = "parameter")
   private List<Parameter> parameters;

   /**
    * Creates a new registry XML token which fields have to be initialized afterwards.
    */
   public Registry()
   {
   }

   /**
    * Creates and initializes a new parameter XML token.
    * <p>
    * The children and parameters are initialized to an empty {@code ArrayList}.
    * </p>
    *
    * @param name the name of this registry.
    */
   public Registry(String name)
   {
      this.name = name;
      registries = new ArrayList<>();
      parameters = new ArrayList<>();
   }

   /**
    * Returns the name of this registry.
    *
    * @return the name of this registry.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Sets the name of this registry.
    *
    * @param name the name of this registry.
    */
   public void setName(String name)
   {
      this.name = name;
   }

   /**
    * Returns the children of this registry.
    *
    * @return the children.
    */
   public List<Registry> getRegistries()
   {
      return registries;
   }

   /**
    * Sets the children of this registry.
    *
    * @param registries the children.
    */
   public void setRegistries(List<Registry> registries)
   {
      this.registries = registries;
   }

   /**
    * Returns the parameters in this registry.
    *
    * @return this registry's parameters.
    */
   public List<Parameter> getParameters()
   {
      return parameters;
   }

   /**
    * Sets the parameters in this registry.
    *
    * @param parameters this registry's parameters.
    */
   public void setParameters(List<Parameter> parameters)
   {
      this.parameters = parameters;
   }
}
