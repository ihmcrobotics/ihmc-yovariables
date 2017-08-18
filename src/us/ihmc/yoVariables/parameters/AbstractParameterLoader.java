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
package us.ihmc.yoVariables.parameters;

import java.util.List;

import javax.xml.stream.events.Namespace;

import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Base class for parameter loaders
 * 
 * @author Jesper Smith
 *
 */
public abstract class AbstractParameterLoader
{
   
   /**
    * Loads all parameters registered to the registry and all its children.
    * 
    * If a parameter cannot be found in the parameter store, it is initialized
    * to its default.
    * 
    * @param registry
    */
   public void loadParametersInRegistry(YoVariableRegistry registry)
   {
      List<YoParameter<?>> parameters = registry.getAllParameters();
      
      for(int i = 0; i < parameters.size(); i++)
      {
         YoParameter<?> parameter = parameters.get(i);
         
         
         NameSpace relativeNamespace = getRelativeNamespace(parameter.getNameSpace(), registry);
         
         if(hasValue(relativeNamespace, parameter.getName()))
         {
            String value = getValue(relativeNamespace, parameter.getName());
            
            
         }
         else
         {
            parameter.setToDefault();
         }
         
         
      }
   }
   
   /**
    * Test if a parameter is available in the parameter value store.
    * 
    * @param namespace Namespace of the parameter
    * @param name Name of the parameter
    * @return true if the value is available in the parameter store
    */
   protected abstract boolean hasValue(NameSpace namespace, String name);
   
   /**
    * Get the value of the parameter.
    * 
    * This function only gets called when hasValue is true.
    * 
    * @param namespace Namespace of the parameter
    * @param name Name of the parameter
    * @return The value of the parameter
    */
   protected abstract String getValue(NameSpace namespace, String name);

   
   protected NameSpace getRelativeNamespace(NameSpace parameterNamespace, YoVariableRegistry registry)
   {
      NameSpace registryNamespace = registry.getNameSpace()
      
      return parameterNamespace.stripOffFromBeginning(registry.getNameSpace());

   }
}
