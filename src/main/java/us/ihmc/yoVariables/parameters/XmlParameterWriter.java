/*
 * Copyright 2017 Florida Institute for Human and Machine Cognition (IHMC) Licensed under the Apache
 * License, Version 2.0 (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied. See the License for the specific language governing permissions and limitations under
 * the License.
 */
package us.ihmc.yoVariables.parameters;

import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.HashMap;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;

import us.ihmc.yoVariables.parameters.xml.Parameter;
import us.ihmc.yoVariables.parameters.xml.Parameters;
import us.ihmc.yoVariables.parameters.xml.Registry;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.tools.YoTools;

public class XmlParameterWriter extends AbstractParameterWriter
{
   private final HashMap<String, Registry> registries = new HashMap<>();
   private final Parameters parameterRoot = new Parameters();

   public XmlParameterWriter()
   {
      parameterRoot.setRegistries(new ArrayList<>());
   }

   private void addNamespace(NameSpace namespace)
   {
      Registry newRegistry = new Registry(YoTools.toShortName(namespace.getName()));

      if (namespace.isRoot())
      {
         parameterRoot.getRegistries().add(newRegistry);
      }
      else
      {
         NameSpace parent = namespace.removeEnd(1);
         if (!registries.containsKey(parent.getName()))
         {
            addNamespace(parent);
         }

         registries.get(parent.getName()).getRegistries().add(newRegistry);
      }

      registries.put(namespace.getName(), newRegistry);

   }

   @Override
   protected void setValue(NameSpace namespace, String name, String description, String type, String value, String min, String max)
   {
      String nameSpaceAsString = namespace.getName();

      if (!registries.containsKey(nameSpaceAsString))
      {
         addNamespace(namespace);
      }

      Parameter newParameter = new Parameter(name, type, value, min, max);
      if (description != null && !description.trim().isEmpty())
      {
         newParameter.setDescription(description);
      }
      else
      {
         newParameter.setDescription(null);
      }

      registries.get(namespace.getName()).getParameters().add(newParameter);
   }

   /**
    * Write the current parameter tree to an OutputStream This function can be called as multiple times
    * without affecting the output.
    *
    * @param os OutputStream to use
    * @throws IOException
    */
   public void write(OutputStream os) throws IOException
   {
      try
      {
         JAXBContext jaxbContext = JAXBContext.newInstance(Parameters.class);
         Marshaller jaxbMarshaller = jaxbContext.createMarshaller();

         jaxbMarshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, true);

         jaxbMarshaller.marshal(parameterRoot, os);
      }
      catch (JAXBException e)
      {
         throw new IOException(e);
      }

   }

}
