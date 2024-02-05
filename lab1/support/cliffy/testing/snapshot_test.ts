import { assert, assertSnapshot } from "../dev_deps.ts";
import { quoteString } from "./_quote_string.ts";
import { dirname, fromFileUrl } from "./deps.ts";

Deno.test({
  name: "should run snapshot tests",
  async fn(ctx) {
    const testDir = dirname(fromFileUrl(import.meta.url));
    const snapshotDir = testDir + "/__snapshots__";
    const snapshotTestDir = testDir + "/__snapshots_test__";

    const snapshotPath = `${snapshotDir}/snapshot_test_fixture.ts.snap`;
    const snapshot2Path = `${snapshotDir}/snapshot_test_fixture_2.ts.snap`;
    const snapshot3Path = `${snapshotTestDir}/snapshot_test_fixture.ts.snap`;

    const args = [
      "test",
      "--allow-run=deno",
      `--allow-read=${testDir}`,
      `--allow-write=${testDir}`,
      "testing/snapshot_test_fixture.ts",
      "--",
      "--update",
    ];

    const cmd = new Deno.Command("deno", { args });

    const { success, stdout, stderr } = await cmd.output();

    const decoder = new TextDecoder();
    assert(success, decoder.decode(stderr) + decoder.decode(stdout));

    const snapshotContent = await Deno.readTextFile(snapshotPath);
    const snapshot2Content = await Deno.readTextFile(snapshot2Path);
    const snapshot3Content = await Deno.readTextFile(snapshot3Path);

    await assertSnapshot(ctx, snapshotContent, { serializer: quoteString });
    await assertSnapshot(ctx, snapshot2Content, { serializer: quoteString });
    await assertSnapshot(ctx, snapshot3Content, { serializer: quoteString });

    await Deno.remove(snapshotPath);
    await Deno.remove(snapshot2Path);
    await Deno.remove(snapshotTestDir, { recursive: true });
  },
});
