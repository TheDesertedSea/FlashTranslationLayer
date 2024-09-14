.PHONY: all clean

$(BUILDDIR)/myFuse.o: $(FUSEDIR)/myFuse.cpp $(HDR) $(CONFIGMK)
	$(vecho) "Compiling $@"
	$(Q)$(CXX) $(CXXFLAGS) -c $< -o $@ `pkg-config fuse --cflags --libs` || \
	{									\
	echo "********************************************************";	\
	echo "*** Make sure ENABLE_LARGE_DATASTORE_PAGE is enabled ***";	\
	echo "********************************************************";	\
	exit 3;									\
	}


$(BUILDDIR)/myFuse: $(OBJ) $(BUILDDIR)/myFuse.o
	$(vecho) "Compiling $@"
	$(Q)$(CXX) $^ `pkg-config fuse --cflags --libs` -o $@


all: $(BUILDDIR)/myFuse
	$(Q)rm -f $(OUTDIR)/myFuse
	$(Q)ln -s $(BUILDDIR)/myFuse $(OUTDIR)/myFuse

clean:
	$(Q)rm -rf $(OUTDIR)/myFuse
	$(Q)rm -rf $(BUILDDIR)/myFuse.o
	$(Q)rm -rf $(BUILDDIR)/myFuse
