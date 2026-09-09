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
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import us.ihmc.yoVariables.parameters.xml.Parameter;
import us.ihmc.yoVariables.parameters.xml.Parameters;
import us.ihmc.yoVariables.parameters.xml.Registry;

/**
 * A parameter reader which manages parameter initialization using a XML file.
 */
public class XmlParameterReader extends AbstractParameterReader
{
   private static final String prefix = "[" + XmlParameterReader.class.getSimpleName() + "]:";

   private final boolean debug;

   private final Map<String, ParameterData> parameterValues = new HashMap<>();

   private final String rootNamespace;

   /**
    * Creates a parameter reader that will read the provided data streams.
    * <p>
    * If more than one data stream is passed to the reader multiple occurrences of the same parameter
    * will cause the parameter value to be overwritten with the new value.
    * </p>
    *
    * @param dataStreams the input streams from which the parameters' value can be read.
    * @throws IOException if something went wrong during parsing.
    */
   public XmlParameterReader(InputStream... dataStreams) throws IOException
   {
      this(false, null, dataStreams);
   }

   /**
    * Creates a parameter reader that will read the provided data streams.
    * <p>
    * If more than one data stream is passed to the reader multiple occurrences of the same parameter
    * will cause the parameter value to be overwritten with the new value.
    * </p>
    *
    * @param rootNamespace allows filtering the data in the data stream this is useful if the provided
    *                      data stream comes from a file that is used to load parameters in multiple
    *                      registries (e.g. controller and estimator).
    * @param dataStreams   the input streams from which the parameters' value can be read.
    * @throws IOException if something went wrong during parsing.
    */
   public XmlParameterReader(String rootNamespace, InputStream... dataStreams) throws IOException
   {
      this(false, rootNamespace, dataStreams);
   }

   /**
    * Creates a parameter reader that will read the provided data streams.
    * <p>
    * If more than one data stream is passed to the reader multiple occurrences of the same parameter
    * will cause the parameter value to be overwritten with the new value.
    * </p>
    *
    * @param debug       specifies whether to print additional information.
    * @param dataStreams the input streams from which the parameters' value can be read.
    * @throws IOException if something went wrong during parsing.
    */
   public XmlParameterReader(boolean debug, InputStream... dataStreams) throws IOException
   {
      this(debug, null, dataStreams);
   }

   /**
    * Creates a parameter reader that will read the provided data streams.
    * <p>
    * If more than one data stream is passed to the reader multiple occurrences of the same parameter
    * will cause the parameter value to be overwritten with the new value.
    * </p>
    *
    * @param debug         specifies whether to print additional information.
    * @param rootNamespace allows filtering the data in the data stream this is useful if the provided
    *                      data stream comes from a file that is used to load parameters in multiple
    *                      registries (e.g. controller and estimator).
    * @param dataStreams   the input streams from which the parameters' value can be read.
    * @throws IOException if something went wrong during parsing.
    */
   public XmlParameterReader(boolean debug, String rootNamespace, InputStream... dataStreams) throws IOException
   {
      this.debug = debug;
      this.rootNamespace = rootNamespace;

      for (InputStream dataStream : dataStreams)
      {
         readStream(dataStream, false);
      }
   }

   /**
    * Will overwrite parameters in the parameter reader with parameters specified in the provided
    * streams. This method will throw a {@link RuntimeException} if any parameter that needs to be
    * overwritten does not exist.
    *
    * @param overwriteParameters the input streams from which the parameters' value to override can be
    *                            read.
    * @throws IOException if something went wrong during parsing.
    */
   public void overwrite(InputStream... overwriteParameters) throws IOException
   {
      for (InputStream dataStream : overwriteParameters)
      {
         readStream(dataStream, true);
      }
   }

   /**
    * Will overwrite parameters in the parameter reader with parameters specified in the provided
    * streams. Unlike {@link #overwrite(InputStream...)}, this method will NOT throw a
    * {@link RuntimeException} if any parameter that needs to be overwritten does not exist, and will
    * instead replace that parameter.
    *
    * @param overwriteParameters the input streams from which the parameters' value to override can be
    *                            read.
    * @throws IOException if something went wrong during parsing.
    */
   public void readAndOverwrite(InputStream... overwriteParameters) throws IOException
   {
      for (InputStream dataStream : overwriteParameters)
      {
         readStream(dataStream, false);
      }
   }

   private void readStream(InputStream data, boolean forceOverwrite) throws IOException
   {
      Parameters parameterRoot = parseParameters(data);

      if (parameterRoot.getRegistries() != null)
      {
         for (Registry registry : parameterRoot.getRegistries())
         {
            if (rootNamespace == null || registry.getName().equals(rootNamespace))
            {
               addRegistry(registry.getName(), registry, forceOverwrite);
            }
         }
      }
   }

   /**
    * Parses the given stream into a {@link Parameters} tree.
    * <p>
    * This is a hand-written replacement for what was previously done via JAXB reflection-based
    * unmarshalling, so that this class stays a straightforward reference for a future non-Java (e.g.
    * C++) port: parse into a DOM tree, then walk it explicitly.
    * </p>
    *
    * @param data the stream containing the XML document to parse.
    * @return the parsed {@link Parameters} tree, with {@code null} lists (as opposed to empty ones)
    *         where the corresponding XML element had no matching children.
    * @throws IOException if the stream could not be parsed as XML.
    */
   private static Parameters parseParameters(InputStream data) throws IOException
   {
      Document document;

      try
      {
         DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
         // Hardening against XXE: this format has no legitimate use for a DOCTYPE or external entities.
         factory.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
         factory.setFeature("http://xml.org/sax/features/external-general-entities", false);
         factory.setFeature("http://xml.org/sax/features/external-parameter-entities", false);
         factory.setXIncludeAware(false);
         factory.setExpandEntityReferences(false);

         DocumentBuilder builder = factory.newDocumentBuilder();
         document = builder.parse(data);
      }
      catch (ParserConfigurationException | SAXException e)
      {
         throw new IOException(e);
      }

      Parameters parameters = new Parameters();
      parameters.setRegistries(parseRegistries(document.getDocumentElement()));
      return parameters;
   }

   private static Registry parseRegistry(Element registryElement)
   {
      Registry registry = new Registry(registryElement.getAttribute("name"));
      registry.setRegistries(parseRegistries(registryElement));
      registry.setParameters(parseParameterList(registryElement));
      return registry;
   }

   private static List<Registry> parseRegistries(Element parent)
   {
      List<Element> registryElements = getChildElementsByTagName(parent, "registry");
      if (registryElements == null)
         return null;

      List<Registry> registries = new ArrayList<>();
      for (Element registryElement : registryElements)
         registries.add(parseRegistry(registryElement));
      return registries;
   }

   private static List<Parameter> parseParameterList(Element parent)
   {
      List<Element> parameterElements = getChildElementsByTagName(parent, "parameter");
      if (parameterElements == null)
         return null;

      List<Parameter> parameters = new ArrayList<>();
      for (Element parameterElement : parameterElements)
         parameters.add(parseParameter(parameterElement));
      return parameters;
   }

   private static Parameter parseParameter(Element parameterElement)
   {
      Parameter parameter = new Parameter(parameterElement.getAttribute("name"),
                                           parameterElement.getAttribute("type"),
                                           getOptionalAttribute(parameterElement, "value"),
                                           getOptionalAttribute(parameterElement, "min"),
                                           getOptionalAttribute(parameterElement, "max"));

      List<Element> descriptionElements = getChildElementsByTagName(parameterElement, "description");
      if (descriptionElements != null)
         parameter.setDescription(descriptionElements.get(0).getTextContent());

      return parameter;
   }

   /**
    * Returns the direct child elements of {@code parent} matching {@code tagName}, or {@code null} if
    * there are none - mirroring JAXB's behavior of leaving a list field {@code null} rather than empty
    * when the corresponding XML elements are absent, which {@link ParameterData}'s null-checks rely on.
    * <p>
    * Deliberately only considers direct children, not all descendants (unlike
    * {@link Element#getElementsByTagName(String)}), since {@code <registry>} elements nest recursively.
    * </p>
    */
   private static List<Element> getChildElementsByTagName(Element parent, String tagName)
   {
      List<Element> result = null;
      NodeList children = parent.getChildNodes();

      for (int i = 0; i < children.getLength(); i++)
      {
         Node child = children.item(i);

         if (child.getNodeType() == Node.ELEMENT_NODE && child.getNodeName().equals(tagName))
         {
            if (result == null)
               result = new ArrayList<>();
            result.add((Element) child);
         }
      }

      return result;
   }

   /** Returns the attribute's value, or {@code null} if {@code element} has no such attribute. */
   private static String getOptionalAttribute(Element element, String attributeName)
   {
      return element.hasAttribute(attributeName) ? element.getAttribute(attributeName) : null;
   }

   private void addRegistry(String path, Registry registry, boolean checkParameterExists)
   {
      if (registry.getParameters() != null)
      {
         for (Parameter param : registry.getParameters())
         {
            String name = path + "." + param.getName();
            ParameterData data = new ParameterData(param.getValue(), param.getMin(), param.getMax());
            if (parameterValues.put(name, data) != null)
            {
               if (debug)
               {
                  System.out.println(prefix + " overwriting " + param.getName());
               }
            }
            else if (checkParameterExists)
            {
               throw new RuntimeException(prefix + " trying to overwrite parameter " + param.getName() + " but it does not exist.");
            }
         }
      }

      if (registry.getRegistries() != null)
      {
         for (Registry child : registry.getRegistries())
         {
            String childPath = path + "." + child.getName();
            addRegistry(childPath, child, checkParameterExists);
         }
      }
   }

   @Override
   protected Map<String, ParameterData> getValues()
   {
      return Collections.unmodifiableMap(parameterValues);
   }
}
