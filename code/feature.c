static EXTENDED_FASTRAM uint32_t activeFeaturesLatch = 0;

void latchActiveFeatures(void)
{
    activeFeaturesLatch = featureConfig()->enabledFeatures;
}

bool featureConfigured(uint32_t mask)
{
    return (featureConfig()->enabledFeatures & mask) == mask;
}

bool feature(uint32_t mask)
{
    // Check for ALL masked features
    return (activeFeaturesLatch & mask) == mask;
}

void featureSet(uint32_t mask)
{
    featureConfigMutable()->enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
    featureConfigMutable()->enabledFeatures &= ~(mask);
}

void featureClearAll(void)
{
    featureConfigMutable()->enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return featureConfig()->enabledFeatures;
}