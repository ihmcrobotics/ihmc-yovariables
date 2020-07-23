package us.ihmc.yoVariables.euclid.referenceFrame.interfaces;

import static gnu.trove.impl.Constants.DEFAULT_CAPACITY;
import static gnu.trove.impl.Constants.DEFAULT_LOAD_FACTOR;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameObject;

/**
 * A map used by the {@link YoMutableFrameObject} to map between frames and frame indices.
 * <p>
 * Several default implementations are provided in this interface.
 * </p>
 * <p>
 * Implementations provided here, relies on {@link ReferenceFrame#getFrameIndex()} to compute a
 * unique index per reference frame. Other implementation may also use
 * {@link ReferenceFrame#hashCode()} instead.
 * </p>
 *
 * @author Georg Wiedebach
 */
public interface FrameIndexMap
{
   /** Key value referencing to the {@code null} value. */
   static final long NO_ENTRY_KEY = -1;

   /**
    * Registers a new reference frame to this map for later use.
    * <p>
    * If the reference frame was already registered, nothing happens.
    * </p>
    * 
    * @param referenceFrame the new reference frame to register.
    */
   public void put(ReferenceFrame referenceFrame);

   /**
    * Retrieves a reference frame previously registered which index correspond
    * 
    * @param frameIndex the index of the reference frame to retrieve. The index is typically obtained
    *                   from {@link ReferenceFrame#getFrameIndex()}.
    * @return the reference frame.
    */
   public ReferenceFrame getReferenceFrame(long frameIndex);

   /**
    * Returns the index for the given reference frame.
    * 
    * @param referenceFrame the reference frame to get the index of.
    * @return the reference frame index value.
    */
   public long getFrameIndex(ReferenceFrame referenceFrame);

   /**
    * Registers all the given reference frames.
    * 
    * @param referenceFrames the collection of reference frames to register.
    * @see #put(ReferenceFrame)
    */
   public default void putAll(Collection<? extends ReferenceFrame> referenceFrames)
   {
      referenceFrames.forEach(referenceFrame -> put(referenceFrame));
   }

   /**
    * Registers all the given reference frames.
    * 
    * @param referenceFrames the list of reference frames to register.
    * @see #put(ReferenceFrame)
    */
   public default void putAll(List<? extends ReferenceFrame> referenceFrames)
   {
      for (int i = 0; i < referenceFrames.size(); i++)
      {
         put(referenceFrames.get(i));
      }
   }

   /**
    * Registers all the given reference frames.
    * 
    * @param referenceFrames the array of reference frames to register.
    * @see #put(ReferenceFrame)
    */
   public default void putAll(ReferenceFrame[] referenceFrames)
   {
      for (int i = 0; i < referenceFrames.length; i++)
      {
         put(referenceFrames[i]);
      }
   }

   /**
    * Implementation of {@code FrameIndexMap} which uses a map internally to store and quickly retrieve
    * reference frames.
    * <p>
    * This implementation is adapted to environments where garbage generation is problematic. Memory is
    * only allocated when registering a reference frame for the first time, and the retrieval is fast
    * and garbage-free.
    * </p>
    * <p>
    * The main restrictions of this implementation are:
    * <ul>
    * <li>it currently does not handle properly removal of a reference frame from its tree;
    * <li>a reference frame must be registered before it can be retrieved.
    * </ul>
    * </p>
    */
   public class FrameIndexHashMap implements FrameIndexMap
   {
      private final TLongObjectMap<ReferenceFrame> frameIndexMap = new TLongObjectHashMap<>(DEFAULT_CAPACITY, DEFAULT_LOAD_FACTOR, NO_ENTRY_KEY);

      @Override
      public void put(ReferenceFrame referenceFrame)
      {
         if (referenceFrame != null)
         {
            frameIndexMap.putIfAbsent(referenceFrame.getFrameIndex(), referenceFrame);
         }
      }

      @Override
      public ReferenceFrame getReferenceFrame(long frameIndex)
      {
         if (frameIndex == NO_ENTRY_KEY)
         {
            return null;
         }
         ReferenceFrame referenceFrame = frameIndexMap.get(frameIndex);
         if (referenceFrame != null)
         {
            return referenceFrame;
         }
         throw new RuntimeException("Can not get the frame for index " + frameIndex + " because the frame index is unknown to the mapper.");
      }

      @Override
      public long getFrameIndex(ReferenceFrame referenceFrame)
      {
         if (referenceFrame == null)
         {
            return NO_ENTRY_KEY;
         }
         if (frameIndexMap.containsValue(referenceFrame))
         {
            return referenceFrame.getFrameIndex();
         }
         throw new RuntimeException("Can not get the frame index for reference frame " + referenceFrame.getName() + " because frame is unknown to the mapper.");
      }
   }

   /**
    * Implementation of {@code FrameIndexMap} which only relies on the reference frame that it was
    * given at construction.
    * <p>
    * This implementation is adapted to environments where there is no restriction on garbage
    * generation. Once created, any reference frame that is a descendant of the root frame (given at
    * construction) can be retrieved. However, any reference frame that is not part of the root frame
    * subtree cannot be retrieved nor registered.
    * </p>
    * <p>
    * The main advantage of this implementation is that it is robust to a dynamically changing
    * reference frame tree. However, reference frame retrieval generates garbage and is less efficient
    * than with {@link FrameIndexHashMap}.
    * </p>
    */
   public class FrameIndexFinder implements FrameIndexMap
   {
      private final ReferenceFrame rootFrame;

      /**
       * Creates a new frame index finder that supports the entire subtree of the given frame and that is
       * ready to use.
       * 
       * @param rootFrame the root frame which tree is to be supported by {@code this}.
       */
      public FrameIndexFinder(ReferenceFrame rootFrame)
      {
         this.rootFrame = rootFrame.getRootFrame();
      }

      @Override
      public void put(ReferenceFrame referenceFrame)
      {
         if (referenceFrame.getRootFrame() != rootFrame)
         {
            throw new RuntimeException("Root frame of " + referenceFrame.getName() + " is " + referenceFrame.getRootFrame().getName()
                  + ". This mapper only supports frame in the tree with root frame " + rootFrame.getName() + ".");
         }
      }

      @Override
      public ReferenceFrame getReferenceFrame(long frameIndex)
      {
         if (frameIndex == NO_ENTRY_KEY)
         {
            return null;
         }
         Collection<ReferenceFrame> allFramesInTree = ReferenceFrameTools.getAllFramesInTree(rootFrame);
         Optional<ReferenceFrame> match = allFramesInTree.stream().filter(referenceFrame -> referenceFrame.getFrameIndex() == frameIndex).findFirst();
         if (match.isPresent())
         {
            return match.get();
         }
         throw new RuntimeException("Can not get the frame for index " + frameIndex + " because the frame index is unknown to the mapper.");
      }

      @Override
      public long getFrameIndex(ReferenceFrame referenceFrame)
      {
         if (referenceFrame == null)
         {
            return NO_ENTRY_KEY;
         }
         return referenceFrame.getFrameIndex();
      }
   }
}
