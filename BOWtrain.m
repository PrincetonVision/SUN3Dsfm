function BOWmodel = BOWtrain(data)

figure
BOWmodel = visualindex_build(data.image, 1:length(data.image), false, 'numWords', 4000) ;


