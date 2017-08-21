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

import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;

import us.ihmc.yoVariables.parameters.xml.Parameter;
import us.ihmc.yoVariables.parameters.xml.Parameters;
import us.ihmc.yoVariables.parameters.xml.Registry;
import us.ihmc.yoVariables.registry.NameSpace;

public class XmlParameterReader extends AbstractParameterReader
{
   private final HashMap<String, String> parameterValues = new HashMap<>();

   public XmlParameterReader(InputStream data) throws IOException
   {
      try
      {
         JAXBContext jaxbContext = JAXBContext.newInstance(Parameters.class);
         Unmarshaller jaxbUnmarshaller = jaxbContext.createUnmarshaller();

         Parameters parameterRoot = (Parameters) jaxbUnmarshaller.unmarshal(data);
         for(Registry registry : parameterRoot.getRegistries())
         {
            addRegistry(registry.getName(), registry);
         }
      }
      catch (JAXBException e)
      {
         throw new IOException(e);
      }

   }

   private void addRegistry(String path, Registry registry)
   {
      if(registry.getParameters() != null)
      {
         for(Parameter param : registry.getParameters())
         {
            String name = path + "." + param.getName();
            parameterValues.put(name, param.getValue());
         }
      }
      
      if(registry.getRegistries() != null)
      {
         for(Registry child : registry.getRegistries())
         {
            String childPath = path + "." + child.getName();
            addRegistry(childPath, child);
         }         
      }
   }

   @Override
   protected boolean hasValue(NameSpace namespace, String name)
   {
      String fullname = namespace.getName() + "." + name;
      return parameterValues.containsKey(fullname);
   }

   @Override
   protected String getValue(NameSpace namespace, String name)
   {
      String fullname = namespace.getName() + "." + name;
      return parameterValues.get(fullname);
   }

}
