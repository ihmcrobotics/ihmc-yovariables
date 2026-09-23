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

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.w3c.dom.Document;
import org.w3c.dom.Element;

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
         DocumentBuilder builder = DocumentBuilderFactory.newInstance().newDocumentBuilder();
         Document document = builder.newDocument();

         Element root = document.createElement("parameters");
         document.appendChild(root);

         for (Registry registry : parameterRoot.getRegistries())
            root.appendChild(toElement(document, registry));

         Transformer transformer = TransformerFactory.newInstance().newTransformer();
         transformer.setOutputProperty(OutputKeys.INDENT, "yes");
         transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2");
         transformer.transform(new DOMSource(document), new StreamResult(outputStream));
      }
      catch (ParserConfigurationException | TransformerException e)
      {
         throw new IOException(e);
      }
   }

   /**
    * Builds a {@code <registry name="..."><registry>...</registry><parameter>...</parameter>
    * </registry>} element for {@code registry}, recursing into its children.
    * <p>
    * Hand-written replacement for what was previously done via JAXB reflection-based marshalling, so
    * this class stays a straightforward reference for a future non-Java (e.g. C++) port: build a DOM
    * tree explicitly, then serialize it.
    * </p>
    */
   private static Element toElement(Document document, Registry registry)
   {
      Element registryElement = document.createElement("registry");
      registryElement.setAttribute("name", registry.getName());

      for (Registry child : registry.getRegistries())
         registryElement.appendChild(toElement(document, child));

      for (Parameter parameter : registry.getParameters())
         registryElement.appendChild(toElement(document, parameter));

      return registryElement;
   }

   private static Element toElement(Document document, Parameter parameter)
   {
      Element parameterElement = document.createElement("parameter");
      parameterElement.setAttribute("name", parameter.getName());
      parameterElement.setAttribute("type", parameter.getType());
      if (parameter.getMin() != null)
         parameterElement.setAttribute("min", parameter.getMin());
      if (parameter.getMax() != null)
         parameterElement.setAttribute("max", parameter.getMax());
      if (parameter.getValue() != null)
         parameterElement.setAttribute("value", parameter.getValue());

      if (parameter.getDescription() != null)
      {
         Element descriptionElement = document.createElement("description");
         descriptionElement.setTextContent(parameter.getDescription());
         parameterElement.appendChild(descriptionElement);
      }

      return parameterElement;
   }
}
