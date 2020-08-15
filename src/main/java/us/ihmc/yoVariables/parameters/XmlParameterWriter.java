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
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.HashMap;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;

import us.ihmc.yoVariables.parameters.xml.Parameter;
import us.ihmc.yoVariables.parameters.xml.Parameters;
import us.ihmc.yoVariables.parameters.xml.Registry;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * A parameter writer which manages parameter export to a XML file.
 */
public class XmlParameterWriter extends AbstractParameterWriter
{
   private final HashMap<String, Registry> registries = new HashMap<>();
   private final Parameters parameterRoot = new Parameters();

   /**
    * Creates a new writer.
    * <p>
    * Before writing parameters into a XML file, it needs to be initialized with the parameters to be
    * exported using {@link #addParameters(YoRegistry)}. This action can be performed on multiple
    * distinct registries which parameters are to be exported. The export can then be finalized with
    * {@link #write(OutputStream)}.
    * </p>
    */
   public XmlParameterWriter()
   {
      parameterRoot.setRegistries(new ArrayList<>());
   }

   private void addNamespace(YoNamespace namespace)
   {
      Registry newRegistry = new Registry(namespace.getShortName());

      if (namespace.isRoot())
      {
         parameterRoot.getRegistries().add(newRegistry);
      }
      else
      {
         YoNamespace parent = namespace.removeEnd(1);
         if (!registries.containsKey(parent.getName()))
         {
            addNamespace(parent);
         }

         registries.get(parent.getName()).getRegistries().add(newRegistry);
      }

      registries.put(namespace.getName(), newRegistry);

   }

   @Override
   protected void setValue(YoNamespace namespace, String name, String description, String type, String value, String min, String max)
   {
      String namespaceAsString = namespace.getName();

      if (!registries.containsKey(namespaceAsString))
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
    * Write the current parameter tree previously initialized via {@link #addParameters(YoRegistry)}.
    *
    * @param outputStream the stream to use for exporting the parameters.
    * @throws IOException if something went wrong during the export process.
    */
   public void write(OutputStream outputStream) throws IOException
   {
      try
      {
         JAXBContext jaxbContext = JAXBContext.newInstance(Parameters.class);
         Marshaller jaxbMarshaller = jaxbContext.createMarshaller();

         jaxbMarshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, true);

         jaxbMarshaller.marshal(parameterRoot, outputStream);
      }
      catch (JAXBException e)
      {
         throw new IOException(e);
      }
   }
}
