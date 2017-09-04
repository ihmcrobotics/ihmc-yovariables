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

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

@XmlAccessorType(XmlAccessType.NONE)
public class Parameter
{
   @XmlAttribute
   private String name;
   
   @XmlAttribute
   private String type;

   @XmlAttribute
   private String min;
   
   @XmlAttribute
   private String max;
   
   @XmlAttribute
   private String value;

   @XmlElement
   private String description;
   
   
   public Parameter()
   {
      
   }
   
   public Parameter(String name, String type, String value, String min, String max)
   {
      this.name = name;
      this.type = type;
      this.value = value;
      this.min = min;
      this.max = max;
   }
   
   
   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public String getType()
   {
      return type;
   }

   public void setType(String type)
   {
      this.type = type;
   }

   public String getValue()
   {
      return value;
   }

   public void setValue(String value)
   {
      this.value = value;
   }

   public String getDescription()
   {
      return description;
   }

   public void setDescription(String description)
   {
      this.description = description;
   }

   public String getMin()
   {
      return min;
   }

   public void setMin(String min)
   {
      this.min = min;
   }

   public String getMax()
   {
      return max;
   }

   public void setMax(String max)
   {
      this.max = max;
   }
   
   
}
