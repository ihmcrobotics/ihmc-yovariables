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
 * This XML token gathers the information for a single parameter.
 * </p>
 */
@XmlAccessorType(XmlAccessType.NONE)
public class Parameter
{
   /** The name of the parameter. */
   @XmlAttribute
   private String name;

   /** The type of the parameter, e.g. "DoubleParameter" or "IntegerParameter". */
   @XmlAttribute
   private String type;

   /** The lower bound associated to the parameter. */
   @XmlAttribute
   private String min;

   /** The upper bound associated to the parameter. */
   @XmlAttribute
   private String max;

   /** The {@code String} representation of the parameter's value. */
   @XmlAttribute
   private String value;

   /** A detailing description associated to the parameter. */
   @XmlElement
   private String description;

   /**
    * Creates a new parameter XML token which fields have to be initialized afterwards.
    */
   public Parameter()
   {
   }

   /**
    * Creates and initializes a new parameter XML token.
    *
    * @param name  the name of the parameter.
    * @param type  the type of the parameter, e.g. "DoubleParameter" or "IntegerParameter", which can
    *              be obtained from the parameter's type simple name, e.g.
    *              {@code DoubleParameter.class.getSimpleName()}.
    * @param value the {@code String} representation of the parameter's value.
    * @param min   the lower bound associated to the parameter.
    * @param max   the upper bound associated to the parameter.
    */
   public Parameter(String name, String type, String value, String min, String max)
   {
      this.name = name;
      this.type = type;
      this.value = value;
      this.min = min;
      this.max = max;
   }

   /**
    * Returns the name of the parameter.
    *
    * @return the name of the parameter.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Sets the name of the parameter.
    *
    * @param name the name of the parameter.
    */
   public void setName(String name)
   {
      this.name = name;
   }

   /**
    * Returns the type of the parameter.
    * <p>
    * For instance, the type can be "DoubleParameter" or "IntegerParameter", which can be obtained from
    * the parameter's type simple name, e.g. {@code DoubleParameter.class.getSimpleName()}.
    * </p>
    *
    * @return the type of the parameter.
    */
   public String getType()
   {
      return type;
   }

   /**
    * Sets the type of the parameter.
    * <p>
    * For instance, the type can be "DoubleParameter" or "IntegerParameter", which can be obtained from
    * the parameter's type simple name, e.g. {@code DoubleParameter.class.getSimpleName()}.
    * </p>
    *
    * @param type the type of the parameter.
    */
   public void setType(String type)
   {
      this.type = type;
   }

   /**
    * Returns the {@code String} representation of the parameter's value.
    *
    * @return the {@code String} representation of the parameter's value.
    */
   public String getValue()
   {
      return value;
   }

   /**
    * Sets the {@code String} representation of the parameter's value.
    *
    * @param value the {@code String} representation of the parameter's value.
    */
   public void setValue(String value)
   {
      this.value = value;
   }

   /**
    * Returns a detailing description associated to the parameter.
    *
    * @return the parameter's description.
    */
   public String getDescription()
   {
      return description;
   }

   /**
    * Sets the detailing description associated to the parameter.
    *
    * @param description the parameter's description.
    */
   public void setDescription(String description)
   {
      this.description = description;
   }

   /**
    * Returns the lower bound associated to the parameter.
    *
    * @return the lower bound associated to the parameter.
    */
   public String getMin()
   {
      return min;
   }

   /**
    * Sets the lower bound associated to the parameter.
    *
    * @param min the lower bound associated to the parameter.
    */
   public void setMin(String min)
   {
      this.min = min;
   }

   /**
    * Returns the upper bound associated to the parameter.
    *
    * @return the upper bound associated to the parameter.
    */
   public String getMax()
   {
      return max;
   }

   /**
    * Sets the upper bound associated to the parameter.
    *
    * @param max the upper bound associated to the parameter.
    */
   public void setMax(String max)
   {
      this.max = max;
   }
}
