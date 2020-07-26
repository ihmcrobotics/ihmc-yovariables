package us.ihmc.yoVariables.tools;

import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * This class provides factories for {@code YoRegistry}.
 */
public class YoFactories
{
   /**
    * First attempts to find a registry with the same namespace as {@code fullNameSpace}, if no such
    * registry could be found then it is created and attached to {@code startRegistry} subtree such
    * that it's namespace equals the given one.
    * 
    * @param startRegistry the registry to start recursing from to look for the namespace.
    * @param fullNameSpace the namespace of the registry to find or create.
    * @return the found/created registry which namespace equals {@code fullNameSpace}, or {@code null}
    *         if the given namespace is incompatible with {@code startRegistry}.
    */
   public static YoRegistry findOrCreateRegistry(YoRegistry startRegistry, NameSpace fullNameSpace)
   {
      NameSpace nameSpace = startRegistry.getNameSpace();

      if (nameSpace.equals(fullNameSpace))
         return startRegistry;

      if (!fullNameSpace.startsWith(nameSpace))
         return null;

      for (YoRegistry child : startRegistry.getChildren())
      {
         YoRegistry registry = findOrCreateRegistry(child, fullNameSpace);
         if (registry != null)
            return registry;
      }

      // If, after going through all the children, none of them match, then
      // create it here and return it.
      NameSpace nameSpaceToContinueWith = fullNameSpace.removeStart(nameSpace);
      YoRegistry registry = createChainOfRegistries(nameSpaceToContinueWith);
      startRegistry.addChild(registry);

      return getToBottomRegistry(registry);
   }

   /**
    * Creates a chain of registries such that the last registry, the one with no child, has its
    * namespace equal to the given one.
    * 
    * @param fullNameSpace the namespace representing the registry chain to create.
    * @return the root registry of the new chain.
    */
   public static YoRegistry createChainOfRegistries(NameSpace fullNameSpace)
   {
      YoRegistry rootRegistry = new YoRegistry(fullNameSpace.getRootName());
      YoRegistry current = rootRegistry;

      for (int i = 1; i < fullNameSpace.size(); i++)
      {
         YoRegistry child = new YoRegistry(fullNameSpace.getSubName(i));
         current.addChild(child);
         current = child;
      }

      return rootRegistry;
   }

   private static YoRegistry getToBottomRegistry(YoRegistry root)
   {
      if (root.getChildren().size() == 0)
         return root;
      if (root.getChildren().size() > 1)
         throw new RuntimeException("This should only be called with a new chain!!");

      return getToBottomRegistry(root.getChildren().get(0));
   }

}
