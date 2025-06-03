#pragma once

#include <esp_partition.h>

void printPartitionTable() {
  esp_partition_iterator_t it = esp_partition_find(
      ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);

  Serial.println("\nPartition Table:");
  Serial.println("Type | Subtype | Offset  | Size    | Label");
  Serial.println("------------------------------------------");

  while (it != NULL) {
    const esp_partition_t *p = esp_partition_get(it);
    Serial.printf("%-4d | %-7d | 0x%05X | 0x%05X | %s\n", p->type, p->subtype,
                  p->address, p->size, p->label);
    it = esp_partition_next(it);
  }
  esp_partition_iterator_release(it);
}

void printSpiffsSFiles() {
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while (file) {
    Serial.printf("File: %s\n", file.name());
    file = root.openNextFile();
  }
}
